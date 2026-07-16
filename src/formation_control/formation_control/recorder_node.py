#!/usr/bin/env python3
"""
recorder_node.py — records the robot camera as a JPEG FRAME SEQUENCE, LOCALLY.

Robust dataset recorder (multi-hour, several robots):
  - Writes the camera's NATIVE JPEG bytes (image_raw/compressed) straight to
    disk: no decode, no re-encode -> the Pi sustains 55-60 fps at full quality.
  - DISK FLOOR (on the robot itself): monitors free space and PAUSES recording
    below `min_free_mb`, resumes when space is freed. This is what actually
    prevents a full disk (a PC-side guard is fragile: it dies with the GUI or a
    dropped SSH). => the disk can no longer hit 0.
  - PER-FRAME TIMESTAMP (ROS stamp + wall) written to frames.csv, plus the
    delta to the previous frame -> lets you align video with the lidar/odom bag
    (same ROS clock) and spot capture gaps.
  - GAP DETECTION: logs a WARN whenever the inter-frame interval exceeds
    `gap_warn_s`, with the exact size and frame index.
  - Segments (folders) rotate every `segment_minutes` so `dataset_tools.sh
    drain` can pull+purge completed segments live.

Layout: ~/dataset/<robot>_<session>_segNN/ frame_000001.jpg ... + frames.csv
  frames.csv: frame, filename, ros_stamp_sec, ros_stamp_nsec, wall_time_iso, dt_s
"""

import os
import time
import csv
import shutil
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder")
        self.declare_parameter("robot_name", "tortuga")
        self.declare_parameter("segment_minutes", 5.0)
        self.declare_parameter("min_free_mb", 700.0)   # pause below this
        self.declare_parameter("gap_warn_s", 0.5)      # warn if gap bigger
        self.declare_parameter("out_dir", os.path.expanduser("~/dataset"))

        self.session = time.strftime("%Y%m%d_%H%M%S")
        self.seg_dir = None
        self.seg_index = 0
        self.seg_start = 0.0
        self.frame_in_seg = 0
        self.frames = 0
        self.cam_count = 0
        self.paused = False
        self.last_stamp = None
        self.csv_file = None
        self.csv_writer = None
        self.last_report_t = time.time()
        self.last_report_frames = 0

        self.out = self.get_parameter("out_dir").value
        os.makedirs(self.out, exist_ok=True)
        # Sensor QoS (BEST_EFFORT): the compressed image topic is BEST_EFFORT.
        self.create_subscription(CompressedImage, "camera/image_raw/compressed",
                                 self.cb, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"Recorder ready (JPEG sequence) -> {self.out} "
            f"(segments {self.get_parameter('segment_minutes').value} min, "
            f"disk floor {self.get_parameter('min_free_mb').value:.0f} MB)")

    def free_mb(self):
        try:
            return shutil.disk_usage(self.out).free / 1e6
        except Exception:
            return None

    def new_segment(self):
        if self.csv_file is not None:
            self.csv_file.close()
        name = self.get_parameter("robot_name").value
        self.seg_index += 1
        self.seg_dir = os.path.join(self.out, f"{name}_{self.session}_seg"
                                    f"{self.seg_index:02d}")
        os.makedirs(self.seg_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.seg_dir, "frames.csv"), "w",
                             newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["frame", "filename", "ros_stamp_sec",
                                  "ros_stamp_nsec", "wall_time_iso", "dt_s"])
        self.frame_in_seg = 0
        self.get_logger().info(f"New segment folder: {self.seg_dir}")

    def cb(self, msg):
        self.cam_count += 1
        now = time.time()

        # --- disk floor (checked ~1x/s) : pause/resume, never fill to 0 ---
        if self.cam_count % 30 == 0:
            fm = self.free_mb()
            floor = float(self.get_parameter("min_free_mb").value)
            if fm is not None:
                if not self.paused and fm < floor:
                    self.get_logger().error(
                        f"LOW DISK {fm:.0f} MB < {floor:.0f} MB -> recording "
                        f"PAUSED. Free space (drain/delete) to resume.")
                    self.paused = True
                elif self.paused and fm > 2 * floor:
                    self.get_logger().warn(
                        f"disk recovered ({fm:.0f} MB) -> recording RESUMED")
                    self.paused = False
        if self.paused:
            return

        seg_s = self.get_parameter("segment_minutes").value * 60.0
        if self.seg_dir is None or (now - self.seg_start) > seg_s:
            self.new_segment()
            self.seg_start = now

        ext = "png" if "png" in (msg.format or "").lower() else "jpg"
        fname = f"frame_{self.frame_in_seg:06d}.{ext}"
        try:
            with open(os.path.join(self.seg_dir, fname), "wb") as f:
                f.write(bytes(msg.data))          # native JPEG bytes, no re-encode
        except Exception as e:
            self.get_logger().warn(f"write failed: {e}")
            return

        # ROS-time gap detection (same clock as the lidar/odom bag -> sync)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = (t - self.last_stamp) if self.last_stamp is not None else 0.0
        if self.last_stamp is not None and dt > float(
                self.get_parameter("gap_warn_s").value):
            self.get_logger().warn(
                f"GAP {dt:.2f}s before frame {self.frame_in_seg} "
                f"(seg {self.seg_index})")
        self.last_stamp = t

        if self.csv_writer is not None:
            self.csv_writer.writerow([self.frame_in_seg, fname,
                                      msg.header.stamp.sec,
                                      msg.header.stamp.nanosec,
                                      time.strftime("%Y-%m-%dT%H:%M:%S"),
                                      f"{dt:.6f}"])
        self.frame_in_seg += 1
        self.frames += 1

    def report(self):
        if self.cam_count == 0:
            self.get_logger().warn(
                "NO image received (QoS/topic?): check the camera publishes on "
                "camera/image_raw/compressed in this namespace.")
            return
        now = time.time()
        dt = now - self.last_report_t
        wrote = self.frames - self.last_report_frames
        self.last_report_t = now
        self.last_report_frames = self.frames
        fm = self.free_mb()
        disk = f"{fm:.0f} MB free" if fm is not None else "disk ?"
        state = " [PAUSED - LOW DISK]" if self.paused else ""
        self.get_logger().info(
            f"segment {self.seg_index}: {self.frames} frames saved "
            f"({self.cam_count} received) | {wrote / dt:.1f} fps | {disk}{state}")

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
        self.get_logger().info(
            f"Final segment closed cleanly ({self.frames} frames total).")


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
