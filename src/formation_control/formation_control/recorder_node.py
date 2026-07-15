#!/usr/bin/env python3
"""
recorder_node.py — records the robot camera as a JPEG FRAME SEQUENCE, LOCALLY.

Why a frame sequence (not a video file):
  The camera already publishes JPEG-compressed frames (image_raw/compressed).
  Re-encoding them into a video on the Raspberry Pi (a) caps out around ~30 fps
  because the CPU can't keep up, and (b) loses quality (double compression).
  Here we write the camera's NATIVE JPEG bytes straight to disk — no decode, no
  re-encode -> the Pi sustains 55-60 fps at full quality. This is also the
  standard format for a detection dataset. Assemble a video offline on the PC
  with tools/assemble_video.sh if you need to watch it.

Layout on the robot:
  ~/dataset/<robot>_<session>_segNN/         (one folder per N-minute segment)
      frame_000001.jpg, frame_000002.jpg, ...
      frames.csv        (frame, filename, ros_stamp_sec, ros_stamp_nsec, wall)

Segments rotate every `segment_minutes` so a brutal kill only affects the last
one, and so `dataset_tools.sh drain` can pull+purge completed segments live.
"""

import os
import time
import csv
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
        self.declare_parameter("out_dir", os.path.expanduser("~/dataset"))

        self.session = time.strftime("%Y%m%d_%H%M%S")
        self.seg_dir = None
        self.seg_index = 0
        self.seg_start = 0.0
        self.frame_in_seg = 0
        self.frames = 0
        self.cam_count = 0
        self.csv_file = None
        self.csv_writer = None
        self.last_report_t = time.time()
        self.last_report_frames = 0

        os.makedirs(self.get_parameter("out_dir").value, exist_ok=True)
        # Sensor QoS (BEST_EFFORT): the compressed image topic is BEST_EFFORT.
        self.create_subscription(CompressedImage, "camera/image_raw/compressed",
                                 self.cb, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"Recorder ready (JPEG sequence) -> {self.get_parameter('out_dir').value} "
            f"(segments of {self.get_parameter('segment_minutes').value} min)")

    def new_segment(self):
        if self.csv_file is not None:
            self.csv_file.close()
        name = self.get_parameter("robot_name").value
        out = self.get_parameter("out_dir").value
        self.seg_index += 1
        self.seg_dir = os.path.join(out, f"{name}_{self.session}_seg"
                                    f"{self.seg_index:02d}")
        os.makedirs(self.seg_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.seg_dir, "frames.csv"), "w",
                             newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["frame", "filename", "ros_stamp_sec",
                                  "ros_stamp_nsec", "wall_time_iso"])
        self.frame_in_seg = 0
        self.get_logger().info(f"New segment folder: {self.seg_dir}")

    def cb(self, msg):
        self.cam_count += 1
        now = time.time()
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
        if self.csv_writer is not None:
            self.csv_writer.writerow([self.frame_in_seg, fname,
                                      msg.header.stamp.sec,
                                      msg.header.stamp.nanosec,
                                      time.strftime("%Y-%m-%dT%H:%M:%S")])
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
        self.get_logger().info(
            f"segment {self.seg_index}: {self.frames} frames saved "
            f"({self.cam_count} received) | {wrote / dt:.1f} fps")

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
        self.get_logger().info("Final segment closed cleanly.")


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
