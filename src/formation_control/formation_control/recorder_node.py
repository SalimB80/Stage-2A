#!/usr/bin/env python3
"""
recorder_node.py — synchronised dataset recorder (camera + lidar + odom), LOCAL.

For each N-minute segment folder it writes, with a PRECISE wall timestamp
(HH:MM:SS.mmm) AND the ROS stamp (same clock across all sensors) so the streams
are directly alignable for an ML dataset:

  ~/dataset/<robot>_<session>_segNN/
      frame_000001.jpg ...            (native JPEG bytes, no re-encode, 55-60fps)
      frames.csv   frame, filename, wall_time, ros_sec, ros_nsec, dt_s
      scan.csv     wall_time, ros_sec, ros_nsec, angle_min, angle_increment,
                   range_min, range_max, ranges (';'-joined)
      odom.csv     wall_time, ros_sec, ros_nsec, x, y, yaw

Robustness:
  - DISK FLOOR on the robot: pauses below `min_free_mb`, resumes when freed ->
    the disk can never hit 0.
  - GAP detection on the camera stream (logs the size + frame index).
  - Clean exit on SIGTERM (segments closed properly).

Align a frame to lidar/pose: match by ros_sec.ros_nsec (exact, same clock) or by
wall_time. Assemble a real-time-paced video from the frames with
tools/assemble_video.sh (uses these timestamps).
"""

import os
import time
import csv
import math
import shutil
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import Odometry


def wall():
    # human-readable wall clock WITH milliseconds -> alignable at 55 fps.
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S.") + \
        f"{datetime.now().microsecond // 1000:03d}"


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder")
        self.declare_parameter("robot_name", "tortuga")
        self.declare_parameter("segment_minutes", 5.0)
        self.declare_parameter("min_free_mb", 700.0)
        self.declare_parameter("gap_warn_s", 0.5)
        self.declare_parameter("out_dir", os.path.expanduser("~/dataset"))

        self.session = time.strftime("%Y%m%d_%H%M%S")
        self.seg_dir = None
        self.seg_index = 0
        self.seg_start = 0.0
        self.frame_in_seg = 0
        self.frames = self.scans = self.odoms = self.cam_count = 0
        self.paused = False
        self.last_stamp = None
        self.f_frames = self.f_scan = self.f_odom = None
        self.w_frames = self.w_scan = self.w_odom = None
        self.last_report_t = time.time()
        self.last_report_frames = 0

        self.out = self.get_parameter("out_dir").value
        os.makedirs(self.out, exist_ok=True)
        q = qos_profile_sensor_data
        self.create_subscription(CompressedImage, "camera/image_raw/compressed",
                                 self.cam_cb, q)
        self.create_subscription(LaserScan, "scan", self.scan_cb, q)
        self.create_subscription(Odometry, "odom", self.odom_cb, q)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"Recorder ready (camera+scan+odom) -> {self.out} "
            f"(segments {self.get_parameter('segment_minutes').value} min, "
            f"disk floor {self.get_parameter('min_free_mb').value:.0f} MB)")

    # ---------- segment / disk ----------
    def free_mb(self):
        try:
            return shutil.disk_usage(self.out).free / 1e6
        except Exception:
            return None

    def disk_ok(self):
        """Return True if we may write; pause/resume around min_free_mb."""
        if self.cam_count % 30 != 0:
            return not self.paused
        fm = self.free_mb()
        floor = float(self.get_parameter("min_free_mb").value)
        if fm is not None:
            if not self.paused and fm < floor:
                self.get_logger().error(
                    f"LOW DISK {fm:.0f} MB < {floor:.0f} MB -> recording PAUSED. "
                    f"Free space (drain/delete) to resume.")
                self.paused = True
            elif self.paused and fm > 2 * floor:
                self.get_logger().warn(
                    f"disk recovered ({fm:.0f} MB) -> recording RESUMED")
                self.paused = False
        return not self.paused

    def ensure_segment(self, now):
        seg_s = self.get_parameter("segment_minutes").value * 60.0
        if self.seg_dir is not None and (now - self.seg_start) <= seg_s:
            return
        for f in (self.f_frames, self.f_scan, self.f_odom):
            if f is not None:
                f.close()
        name = self.get_parameter("robot_name").value
        self.seg_index += 1
        self.seg_dir = os.path.join(self.out, f"{name}_{self.session}_seg"
                                    f"{self.seg_index:02d}")
        os.makedirs(self.seg_dir, exist_ok=True)
        self.f_frames = open(os.path.join(self.seg_dir, "frames.csv"), "w", newline="")
        self.f_scan = open(os.path.join(self.seg_dir, "scan.csv"), "w", newline="")
        self.f_odom = open(os.path.join(self.seg_dir, "odom.csv"), "w", newline="")
        self.w_frames = csv.writer(self.f_frames)
        self.w_scan = csv.writer(self.f_scan)
        self.w_odom = csv.writer(self.f_odom)
        self.w_frames.writerow(["frame", "filename", "wall_time",
                                "ros_sec", "ros_nsec", "dt_s"])
        self.w_scan.writerow(["wall_time", "ros_sec", "ros_nsec", "angle_min",
                              "angle_increment", "range_min", "range_max", "ranges"])
        self.w_odom.writerow(["wall_time", "ros_sec", "ros_nsec", "x", "y", "yaw"])
        self.frame_in_seg = 0
        self.seg_start = now
        self.get_logger().info(f"New segment: {self.seg_dir}")

    # ---------- callbacks ----------
    def cam_cb(self, msg):
        self.cam_count += 1
        now = time.time()
        if not self.disk_ok():
            return
        self.ensure_segment(now)
        ext = "png" if "png" in (msg.format or "").lower() else "jpg"
        fname = f"frame_{self.frame_in_seg:06d}.{ext}"
        try:
            with open(os.path.join(self.seg_dir, fname), "wb") as f:
                f.write(bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"write failed: {e}")
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = (t - self.last_stamp) if self.last_stamp is not None else 0.0
        if self.last_stamp is not None and dt > float(
                self.get_parameter("gap_warn_s").value):
            self.get_logger().warn(
                f"GAP {dt:.2f}s before frame {self.frame_in_seg} "
                f"(seg {self.seg_index})")
        self.last_stamp = t
        self.w_frames.writerow([self.frame_in_seg, fname, wall(),
                                msg.header.stamp.sec, msg.header.stamp.nanosec,
                                f"{dt:.6f}"])
        self.frame_in_seg += 1
        self.frames += 1

    def scan_cb(self, msg):
        if self.seg_dir is None or self.paused:
            return
        ranges = ";".join(f"{r:.3f}" for r in msg.ranges)
        self.w_scan.writerow([wall(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                              f"{msg.angle_min:.6f}", f"{msg.angle_increment:.6f}",
                              f"{msg.range_min:.3f}", f"{msg.range_max:.3f}", ranges])
        self.scans += 1

    def odom_cb(self, msg):
        if self.seg_dir is None or self.paused:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.w_odom.writerow([wall(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                              f"{p.x:.4f}", f"{p.y:.4f}", f"{yaw:.4f}"])
        self.odoms += 1

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
            f"seg {self.seg_index}: {self.frames} frames ({wrote / dt:.1f} fps), "
            f"{self.scans} scans, {self.odoms} odom | {disk}{state}")

    def close(self):
        for f in (self.f_frames, self.f_scan, self.f_odom):
            if f is not None:
                f.close()
        self.get_logger().info(
            f"Final segment closed cleanly ({self.frames} frames, "
            f"{self.scans} scans, {self.odoms} odom).")


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
