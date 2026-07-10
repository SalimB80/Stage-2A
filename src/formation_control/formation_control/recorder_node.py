#!/usr/bin/env python3
"""
recorder_node.py — enregistre la camera du robot en video, LOCALEMENT.

- Ecrit dans ~/dataset/ sur le robot (pas de streaming reseau).
- Segments de N minutes (defaut 5) : un kill brutal ne corrompt que le
  dernier segment, les precedents restent lisibles.
- Codec mp4v (leger, ~1-2 Go/h) avec repli MJPG/avi si mp4v indisponible.
- Nom : tortugaX_AAAAMMJJ_HHMMSS_segNN.mp4
"""

import os
import time
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder")
        self.declare_parameter("robot_name", "tortuga")
        self.declare_parameter("segment_minutes", 5.0)
        self.declare_parameter("fps", 15.0)   # metadonnee de lecture
        self.declare_parameter("out_dir", os.path.expanduser("~/dataset"))

        self.bridge = CvBridge()
        self.writer = None
        self.csv_file = None
        self.csv_writer = None
        self.seg_start = 0.0
        self.seg_index = 0
        self.frames = 0
        self.size = None
        self.session = time.strftime("%Y%m%d_%H%M%S")

        self.cam_count = 0

        os.makedirs(self.get_parameter("out_dir").value, exist_ok=True)
        # QoS capteur (BEST_EFFORT) : la camera publie en BEST_EFFORT. Avec un
        # abonne RELIABLE (defaut) on ne recevait AUCUNE image -> videos vides.
        self.create_subscription(Image, "camera/image_raw", self.cb,
                                 qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"Recorder pret -> {self.get_parameter('out_dir').value} "
            f"(segments de {self.get_parameter('segment_minutes').value} min)")

    def new_writer(self, w, h):
        name = self.get_parameter("robot_name").value
        out = self.get_parameter("out_dir").value
        fps = float(self.get_parameter("fps").value)
        self.seg_index += 1
        base = f"{name}_{self.session}_seg{self.seg_index:02d}"

        path = os.path.join(out, base + ".mp4")
        wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (w, h))
        if not wr.isOpened():                      # repli MJPG/avi
            path = os.path.join(out, base + ".avi")
            wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"),
                                 fps, (w, h))
        if not wr.isOpened():
            self.get_logger().error("Impossible d'ouvrir un VideoWriter !")
            return None
        # CSV d'horodatage : frame -> timestamp ROS + heure murale.
        # (On n'incruste PAS l'heure dans l'image : cela polluerait le
        # dataset d'entrainement. Le CSV suffit pour dater chaque frame.)
        if self.csv_file is not None:
            self.csv_file.close()
        csv_path = os.path.splitext(path)[0] + ".csv"
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["frame", "ros_stamp_sec", "ros_stamp_nsec",
                                  "wall_time_iso"])
        self.frame_in_seg = 0
        self.get_logger().info(f"Nouveau segment : {path} (+ {csv_path})")
        return wr

    def cb(self, msg):
        self.cam_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge: {e}")
            return
        h, w = frame.shape[:2]
        now = time.time()
        seg_s = self.get_parameter("segment_minutes").value * 60.0

        if self.writer is None or (now - self.seg_start) > seg_s \
           or self.size != (w, h):
            if self.writer is not None:
                self.writer.release()
            self.writer = self.new_writer(w, h)
            self.seg_start = now
            self.size = (w, h)
        if self.writer is not None:
            self.writer.write(frame)
            self.frames += 1
            if self.csv_writer is not None:
                self.csv_writer.writerow([
                    self.frame_in_seg,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    time.strftime("%Y-%m-%dT%H:%M:%S")])
                self.frame_in_seg += 1

    def report(self):
        if self.cam_count == 0:
            self.get_logger().warn(
                "AUCUNE image recue (QoS/topic ?) : verifie que la camera "
                "publie sur camera/image_raw dans ce namespace.")
            return
        self.get_logger().info(
            f"{self.frames} images ecrites (segment {self.seg_index}), "
            f"{self.cam_count} recues au total")

    def close(self):
        if self.writer is not None:
            self.writer.release()
        if self.csv_file is not None:
            self.csv_file.close()
        self.get_logger().info("Segment final ferme proprement.")


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
