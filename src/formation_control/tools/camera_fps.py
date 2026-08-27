"""
camera_fps
==========

A small Python library to change the frame rate of a camera driven by the
`camera_ros` node (which uses libcamera underneath). Developed for Raspberry Pi
4B and Pi 5; the mechanism itself is board-independent and should work with any
`camera_ros` node that exposes `FrameDurationLimits`.

How it works
------------
libcamera does not expose an "fps" setting directly. Instead, frame rate is
governed by the ``FrameDurationLimits`` control: a pair of integers
``[min_duration_us, max_duration_us]`` giving the allowed frame duration in
microseconds. `camera_ros` exposes the camera's libcamera controls as ROS 2
parameter on the camera node, so changing fps at runtime is done by calling
the node's ``set_parameters`` service with:

    FrameDurationLimits = [1e6 / fps, 1e6 / fps]

Locking min == max forces a fixed frame rate (as long as the requested value
is within the sensor mode's supported range and the exposure time fits).

Exposure caveat
---------------
Exposure time can never exceed frame duration. If the auto-exposure algorithm
wants a 40 ms exposure but you lock to 30 fps (33.3 ms/frame), the request is
physically impossible, and — depending on the camera pipeline and AE state —
libcamera may shorten the exposure (brightness handled by gain, so the rate
holds), let the frame duration stretch so the effective rate drops below your
target, or reject the control set. This is not a library bug; see
HOW_IT_WORKS.md. Use ``lock=False`` when a correct exposure matters more than a
perfectly steady rate.

Usage
-----
Standalone (the library manages rclpy itself)::

    from camera_fps import CameraFPSController

    with CameraFPSController(camera_node="/camera/camera") as cam:
        cam.set_fps(15.0)
        print(cam.get_fps())

Inside an existing rclpy node (an executor must be spinning it)::

    ctrl = CameraFPSController(camera_node="/camera/camera", node=my_node)
    ctrl.set_fps(30.0)

Command line::

    python3 camera_fps.py --node /camera/camera --fps 15
    python3 camera_fps.py --node /camera/camera --get
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    SetParameters,
)
from rcl_interfaces.msg import (
    Parameter,
    ParameterDescriptor,
    ParameterValue,
    ParameterType,
)

__all__ = ["CameraFPSController", "CameraFPSError"]

# Name of the libcamera control exposed by camera_ros as a ROS parameter.
FRAME_DURATION_PARAM = "FrameDurationLimits"
# Conversion constant: one second expressed in microseconds.
MICROSECONDS_PER_SECOND = 1_000_000


class CameraFPSError(RuntimeError):
    """Raised when talking to the camera node fails."""


class CameraFPSController:
    """Change / query the fps of a camera_ros node at runtime.

    Parameters
    ----------
    camera_node:
        Fully qualified name of the camera node, e.g. ``/camera/camera``.
        (With default launch files camera_ros runs as node ``camera`` in
        namespace ``camera``.)
    node:
        Optional existing rclpy node to piggyback on. If ``None``, the
        controller creates (and later destroys) its own helper node and,
        if needed, initializes rclpy. If you pass a node, an executor must
        be spinning it elsewhere for calls to complete.
    timeout_sec:
        Timeout for service discovery and calls.
    """

    def __init__(
        self,
        camera_node: str = "/camera/camera",
        node: Optional[Node] = None,
        timeout_sec: float = 5.0,
    ) -> None:
        self._camera_node = camera_node.rstrip("/")
        self._timeout = timeout_sec
        self._owns_rclpy = False
        self._owns_node = node is None

        if node is None:
            if not rclpy.ok():
                rclpy.init()
                self._owns_rclpy = True
            node = rclpy.create_node("camera_fps_controller")
        self._node = node

        # Store service names explicitly rather than relying on the client's
        # internal ``srv_name`` attribute.
        self._set_srv = f"{self._camera_node}/set_parameters"
        self._get_srv = f"{self._camera_node}/get_parameters"
        self._desc_srv = f"{self._camera_node}/describe_parameters"

        self._set_cli = self._node.create_client(SetParameters, self._set_srv)
        self._get_cli = self._node.create_client(GetParameters, self._get_srv)
        self._desc_cli = self._node.create_client(
            DescribeParameters, self._desc_srv
        )

        # Cache of which services have been confirmed available (services do
        # not normally disappear once seen, so we only wait_for_service once).
        self._confirmed: set[str] = set()
        # Cache of the FrameDurationLimits parameter descriptor. The sensor
        # mode is fixed for the node's lifetime, so the descriptor (and thus
        # the supported window and the parameter type) does not change.
        self._descriptor: Optional[ParameterDescriptor] = None
        self._descriptor_fetched = False

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def set_fps(
        self,
        fps: float,
        lock: bool = True,
        check: bool = True,
        verify: bool = False,
        verify_rel_tol: float = 0.02,
    ) -> Optional[float]:
        """Set the camera frame rate.

        Parameters
        ----------
        fps:
            Target frame rate in frames per second. Must be > 0.
        lock:
            If True (default), min and max frame duration are both set to
            1e6/fps, forcing a fixed rate. If False, the minimum duration is
            constrained (up to ``fps``) while the maximum is left at the
            sensor mode's own maximum frame duration, so the camera may run
            *up to* ``fps`` but drop lower if exposure requires it.
        check:
            If True (default), validate the target against the current sensor
            mode's supported fps window (when the driver exposes it) and raise
            a clear error before calling the service. On the Pi the ceiling is
            set by the launched resolution/mode, so a target above it can only
            be reached by launching a different (usually lower-res) mode.
        verify:
            If True, read the FrameDurationLimits back after setting and raise
            ``CameraFPSError`` if the *applied limits* differ from what was
            requested by more than ``verify_rel_tol`` (catching drivers that
            silently clamp to the nearest supported value). This verifies the
            applied limits, NOT the live running frame rate: with ``lock=False``
            the actual rate can be lower than the cap whenever AE lengthens the
            exposure, and that can only be measured downstream (e.g.
            ``ros2 topic hz /camera/image_raw``). Returns the applied fps cap
            (1e6 / applied_min_us) when set.
        verify_rel_tol:
            Relative (fractional) tolerance for the read-back comparison
            (default 0.02 = 2%).

        Returns
        -------
        The applied fps cap (read back) when ``verify=True``, otherwise
        ``None``.
        """
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")

        # Fetch the mode's supported frame-duration window (microseconds) at
        # most once: needed for the range check and, when unlocking, for the
        # max frame duration. Working in µs throughout avoids a round-trip
        # through fps.
        limits_us = None
        if check or not lock:
            limits_us = self.get_supported_frame_duration_limits()

        if check and limits_us is not None:
            mode_min_us, mode_max_us = limits_us
            hi_fps = MICROSECONDS_PER_SECOND / mode_min_us  # fastest allowed
            lo_fps = MICROSECONDS_PER_SECOND / mode_max_us  # slowest allowed
            # Small tolerance for sensor quantisation.
            if fps > hi_fps * 1.001 or fps < lo_fps * 0.999:
                raise CameraFPSError(
                    f"{fps} fps is outside the current sensor mode's "
                    f"range ({lo_fps:.2f}-{hi_fps:.2f} fps). "
                    f"To go higher, relaunch camera_ros with a "
                    f"lower-resolution sensor mode (smaller width/height); "
                    f"to go lower than the minimum, that mode simply can't "
                    f"run that slow. Pass check=False to try anyway."
                )

        duration_us = int(round(MICROSECONDS_PER_SECOND / fps))
        if lock:
            min_us, max_us = duration_us, duration_us
        else:
            min_us = duration_us
            max_us = self._unlock_max_duration_us(duration_us, limits_us)

        self.set_frame_duration_limits(min_us, max_us)

        if verify:
            applied_min, applied_max = self.get_frame_duration_limits()
            # The requested cap (fastest allowed) corresponds to min_us.
            if abs(applied_min - min_us) > min_us * verify_rel_tol:
                raise CameraFPSError(
                    f"Requested an fps cap of "
                    f"{MICROSECONDS_PER_SECOND / min_us:.3f} fps "
                    f"(min duration {min_us} us) but the camera applied "
                    f"{MICROSECONDS_PER_SECOND / applied_min:.3f} fps "
                    f"({applied_min} us). The driver likely clamped to its "
                    f"nearest supported value."
                )
            if lock and abs(applied_max - max_us) > max_us * verify_rel_tol:
                raise CameraFPSError(
                    f"Lock not honoured: requested max duration {max_us} us "
                    f"but the camera applied {applied_max} us, so the rate is "
                    f"not pinned."
                )
            return MICROSECONDS_PER_SECOND / applied_min
        return None

    def get_fps(self) -> float:
        """Return the current fps, derived from FrameDurationLimits.

        Uses the *maximum* frame duration (i.e. the guaranteed minimum
        rate). If min and max differ, the camera runs somewhere between
        1e6/max and 1e6/min fps.
        """
        lo, hi = self.get_frame_duration_limits()
        if hi <= 0:
            raise CameraFPSError(f"Invalid FrameDurationLimits: [{lo}, {hi}]")
        return MICROSECONDS_PER_SECOND / hi

    def set_frame_duration_limits(self, min_us: int, max_us: int) -> None:
        """Set FrameDurationLimits (microseconds) directly."""
        if min_us > max_us:
            raise ValueError("min_us must be <= max_us")

        # If the driver exposes a descriptor, confirm the control is still an
        # integer array before we build the request, so a driver change gives
        # a clear error here rather than an opaque service rejection.
        descriptor = self._get_descriptor()
        if descriptor is not None and \
                descriptor.type != ParameterType.PARAMETER_INTEGER_ARRAY:
            raise CameraFPSError(
                f"'{FRAME_DURATION_PARAM}' is no longer an integer array "
                f"(descriptor type={descriptor.type}); this driver version is "
                f"not supported by camera_fps."
            )

        param = Parameter(
            name=FRAME_DURATION_PARAM,
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[int(min_us), int(max_us)],
            ),
        )
        request = SetParameters.Request(parameters=[param])
        response = self._call(self._set_cli, self._set_srv, request)

        result = response.results[0]
        if not result.successful:
            raise CameraFPSError(
                f"Camera node rejected FrameDurationLimits "
                f"[{min_us}, {max_us}]: {result.reason or 'no reason given'}"
            )

    def get_supported_frame_duration_limits(self) -> Optional[Tuple[int, int]]:
        """Return the (min_us, max_us) frame durations the current mode allows.

        This is the native libcamera representation: the shortest and longest
        frame duration (microseconds) permitted by the sensor mode that was
        selected when camera_ros launched. Read from the ``FrameDurationLimits``
        parameter descriptor's integer range.

        Returns ``None`` if the driver did not populate the range (older
        camera_ros builds may not).
        """
        descriptor = self._get_descriptor()
        if descriptor is None or not descriptor.integer_range:
            return None
        # The descriptor message allows multiple integer ranges, but camera_ros
        # publishes a single range for FrameDurationLimits, so we use the first.
        r = descriptor.integer_range[0]
        # r.from_value / r.to_value are frame durations in microseconds.
        if r.from_value <= 0 or r.to_value <= 0:
            return None
        return int(r.from_value), int(r.to_value)

    def get_supported_fps_range(self) -> Optional[Tuple[float, float]]:
        """Return the (min_fps, max_fps) achievable in the *current* sensor mode.

        A thin conversion of :meth:`get_supported_frame_duration_limits` into
        frames per second, so you can validate an fps *before* trying to set
        it. On the Pi the window is fixed by the resolution/mode chosen at
        launch.

        Returns ``None`` if the driver did not report the range.
        """
        limits = self.get_supported_frame_duration_limits()
        if limits is None:
            return None
        min_us, max_us = limits
        min_fps = MICROSECONDS_PER_SECOND / max_us   # longest -> slowest
        max_fps = MICROSECONDS_PER_SECOND / min_us   # shortest -> fastest
        return (min_fps, max_fps)

    def get_frame_duration_limits(self) -> Tuple[int, int]:
        """Return the current FrameDurationLimits as (min_us, max_us)."""
        request = GetParameters.Request(names=[FRAME_DURATION_PARAM])
        response = self._call(self._get_cli, self._get_srv, request)

        if not response.values:
            raise CameraFPSError(
                f"Parameter '{FRAME_DURATION_PARAM}' not found on "
                f"{self._camera_node}"
            )
        value = response.values[0]
        # FrameDurationLimits is defined as exactly two integers.
        if value.type != ParameterType.PARAMETER_INTEGER_ARRAY or \
                len(value.integer_array_value) != 2:
            raise CameraFPSError(
                f"'{FRAME_DURATION_PARAM}' has unexpected type/shape "
                f"(type={value.type}, "
                f"len={len(value.integer_array_value)}); is the camera "
                f"streaming and does the sensor support this control?"
            )
        arr = value.integer_array_value
        return int(arr[0]), int(arr[1])

    def close(self) -> None:
        """Release resources created by this controller."""
        self._node.destroy_client(self._set_cli)
        self._node.destroy_client(self._get_cli)
        self._node.destroy_client(self._desc_cli)
        if self._owns_node:
            self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------ #
    # Context manager support
    # ------------------------------------------------------------------ #

    def __enter__(self) -> "CameraFPSController":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _get_descriptor(self) -> Optional[ParameterDescriptor]:
        """Fetch and cache the FrameDurationLimits parameter descriptor.

        The sensor mode is fixed for the node's lifetime, so this is fetched
        once and cached for the life of the controller.
        """
        if self._descriptor_fetched:
            return self._descriptor
        request = DescribeParameters.Request(names=[FRAME_DURATION_PARAM])
        response = self._call(self._desc_cli, self._desc_srv, request)
        self._descriptor = (
            response.descriptors[0] if response.descriptors else None
        )
        self._descriptor_fetched = True
        return self._descriptor

    def _unlock_max_duration_us(
        self, requested_min_us: int, limits_us: Optional[Tuple[int, int]]
    ) -> int:
        """Pick the max frame duration for ``lock=False``.

        Prefer the sensor mode's own maximum (from the already-fetched
        ``limits_us`` (min_us, max_us) pair) so the camera can slow down
        exactly as far as the mode allows; fall back to the currently-set max,
        and only as a last resort to a 1-second (1 fps) floor. Never returns
        less than the requested minimum. Operates entirely in microseconds.
        """
        max_us: Optional[int] = None
        if limits_us is not None:
            _, max_us = limits_us
        if max_us is None:
            try:
                _, current_max = self.get_frame_duration_limits()
                max_us = current_max
            except CameraFPSError:
                max_us = MICROSECONDS_PER_SECOND
        return max(max_us, requested_min_us)

    def _call(self, client, srv_name: str, request: Any) -> Any:
        # Only wait for the service the first time; once seen it is cached.
        if srv_name not in self._confirmed:
            if not client.wait_for_service(timeout_sec=self._timeout):
                raise CameraFPSError(
                    f"Service '{srv_name}' not available after "
                    f"{self._timeout}s. Is the camera_ros node running and is "
                    f"'{self._camera_node}' the correct node name? "
                    f"(check with: ros2 node list)"
                )
            self._confirmed.add(srv_name)

        future = client.call_async(request)

        # If we own the helper node we spin it ourselves. If we are attached
        # to a user-supplied node, we rely on that node's executor (running
        # elsewhere, typically another thread) to complete the future.
        if self._owns_node:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self._timeout
            )
        else:
            deadline = time.monotonic() + self._timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)

        if not future.done():
            if self._owns_node:
                raise CameraFPSError(
                    f"Call to '{srv_name}' timed out after {self._timeout}s; "
                    f"the camera node may be unresponsive."
                )
            raise CameraFPSError(
                f"Call to '{srv_name}' did not complete within "
                f"{self._timeout}s. When attached to an existing node, "
                f"camera_fps depends on YOUR executor to spin that node — the "
                f"response can only arrive while the node is being spun (e.g. "
                f"rclpy.spin(node), or a MultiThreadedExecutor running in "
                f"another thread). If nothing is spinning the node, this call "
                f"can never complete. This is an executor problem, not a "
                f"network one."
            )
        if future.result() is None:
            raise CameraFPSError(
                f"Call to '{srv_name}' failed: {future.exception()}"
            )
        return future.result()


# ---------------------------------------------------------------------- #
# CLI
# ---------------------------------------------------------------------- #

def _fps_summary(lo: int, hi: int) -> str:
    """Human-readable fps for a [min_us, max_us] pair."""
    if lo == hi:
        return f"{MICROSECONDS_PER_SECOND / hi:.2f} fps"
    # lo is min duration (fastest); hi is max duration (slowest).
    return (f"{MICROSECONDS_PER_SECOND / hi:.2f}-"
            f"{MICROSECONDS_PER_SECOND / lo:.2f} fps range")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Set or query the fps of a camera_ros node."
    )
    parser.add_argument(
        "--node", default="/camera/camera",
        help="Fully qualified camera node name (default: /camera/camera)",
    )
    parser.add_argument("--fps", type=float, help="Target fps to set")
    parser.add_argument(
        "--no-lock", action="store_true",
        help="Only cap the fps (allow the camera to run slower if needed)",
    )
    parser.add_argument(
        "--get", action="store_true", help="Print the current fps"
    )
    parser.add_argument(
        "--range", action="store_true",
        help="Print the fps window supported by the current sensor mode",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Skip the sensor-mode range check when setting fps",
    )
    args = parser.parse_args(argv)

    if args.fps is None and not args.get and not args.range:
        parser.error("nothing to do: pass --fps, --get and/or --range")

    try:
        with CameraFPSController(camera_node=args.node) as cam:
            if args.range:
                window = cam.get_supported_fps_range()
                if window is None:
                    print("supported fps range: not reported by this driver")
                else:
                    print(f"supported fps range (current mode): "
                          f"{window[0]:.2f}-{window[1]:.2f} fps")
            limits = None  # (lo, hi) cache to avoid a second get_parameters
            if args.fps is not None:
                cam.set_fps(args.fps, lock=not args.no_lock, check=not args.force)
                lo, hi = cam.get_frame_duration_limits()
                limits = (lo, hi)
                print(f"Requested: {args.fps:.2f} fps")
                print(f"Applied:   {_fps_summary(lo, hi)}")
                print(f"FrameDurationLimits = [{lo}, {hi}] us")
            if args.get:
                if limits is None:
                    limits = cam.get_frame_duration_limits()
                lo, hi = limits
                print(f"FrameDurationLimits: [{lo}, {hi}] us "
                      f"(~{_fps_summary(lo, hi)})")
    except (CameraFPSError, ValueError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
