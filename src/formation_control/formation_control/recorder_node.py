#!/usr/bin/env python3
"""
recorder_node.py — enregistre la camera du robot en video, LOCALEMENT.

Objectif dataset (detection de robots) : capturer la camera a la
FREQUENCE MAXIMALE, chaque robot filmant les autres qui bougent.

- Ecrit dans ~/dataset/ sur le robot (pas de streaming reseau).
- Segments de N minutes (defaut 5) : un kill brutal ne corrompt que le
  dernier segment, les precedents restent lisibles.
- Codec MJPG par defaut : tres leger a encoder -> soutient un FPS eleve
  sur Raspberry Pi sans perdre d'images (fichiers plus gros, assume pour
  un dataset). Repli mp4v possible via le parametre 'codec'.
- FPS AUTO-MESURE : on mesure la cadence reelle de la camera sur les
  premieres images, puis on ecrit la video a cette cadence -> relecture a
  la bonne vitesse. Le CSV d'horodatage reste la verite terrain.
- Nom : tortugaX_AAAAMMJJ_HHMMSS_segNN.avi (+ .csv d'horodatage par frame)
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
        self.declare_parameter("fps", 60.0)      # FALLBACK si mesure indispo
        self.declare_parameter("codec", "MJPG")  # MJPG (leger, max fps) | mp4v
        self.declare_parameter("measure_frames", 40)  # images pour estimer le fps
        self.declare_parameter("out_dir", os.path.expanduser("~/dataset"))

        self.bridge = CvBridge()
        self.writer = None
        self.csv_file = None
        self.csv_writer = None
        self.seg_start = 0.0
        self.seg_index = 0
        self.frames = 0
        self.frame_in_seg = 0
        self.size = None
        self.session = time.strftime("%Y%m%d_%H%M%S")

        # --- diagnostic / mesure FPS ---
        self.cam_count = 0
        self.fps_measured = None      # cadence reelle (img/s), calculee au demarrage
        self.meas_t0 = None
        self.meas_n = 0
        self.last_report_t = time.time()
        self.last_report_frames = 0

        os.makedirs(self.get_parameter("out_dir").value, exist_ok=True)
        # QoS capteur (BEST_EFFORT) : la camera publie en BEST_EFFORT. Avec un
        # abonne RELIABLE (defaut) on ne recevait AUCUNE image -> videos vides.
        self.create_subscription(Image, "camera/image_raw", self.cb,
                                 qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"Recorder pret -> {self.get_parameter('out_dir').value} "
            f"(segments de {self.get_parameter('segment_minutes').value} min, "
            f"codec {self.get_parameter('codec').value}, mesure du FPS en cours...)")

    def new_writer(self, w, h):
        name = self.get_parameter("robot_name").value
        out = self.get_parameter("out_dir").value
        # cadence d'ecriture = FPS reellement mesure (sinon fallback parametre)
        fps = self.fps_measured or float(self.get_parameter("fps").value)
        codec = str(self.get_parameter("codec").value).upper()
        self.seg_index += 1
        base = f"{name}_{self.session}_seg{self.seg_index:02d}"

        # MJPG -> .avi (leger a encoder, ideal haut FPS) ; mp4v -> .mp4 (compact)
        if codec == "MP4V":
            trials = [("mp4v", ".mp4"), ("MJPG", ".avi")]
        else:
            trials = [("MJPG", ".avi"), ("mp4v", ".mp4")]

        wr, path = None, None
        for fourcc, ext in trials:
            path = os.path.join(out, base + ext)
            wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc),
                                 fps, (w, h))
            if wr.isOpened():
                break
        if wr is None or not wr.isOpened():
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
        self.get_logger().info(
            f"Nouveau segment : {path} @ {fps:.1f} fps (+ {csv_path})")
        return wr

    def cb(self, msg):
        self.cam_count += 1
        now = time.time()

        # --- Phase de MESURE du FPS reel (avant tout enregistrement) ---
        # On chronometre l'arrivee des premieres images pour connaitre la
        # cadence effective de la camera, puis on ecrit la video a ce rythme.
        if self.fps_measured is None:
            if self.meas_t0 is None:
                self.meas_t0 = now          # 1re image = t0, non comptee
                self.meas_n = 0
                return
            self.meas_n += 1
            elapsed = now - self.meas_t0
            need = int(self.get_parameter("measure_frames").value)
            if self.meas_n >= need and elapsed > 0.5:
                self.fps_measured = self.meas_n / elapsed
                self.get_logger().info(
                    f"FPS camera mesure : {self.fps_measured:.1f} img/s "
                    f"-> ecriture video a cette cadence.")
            else:
                return   # on n'ecrit pas tant que le fps n'est pas connu

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge: {e}")
            return
        h, w = frame.shape[:2]
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
        # cadence d'ecriture reellement atteinte depuis le dernier report :
        # si elle chute nettement sous le FPS mesure, le Pi perd des images.
        now = time.time()
        dt = now - self.last_report_t
        dframes = self.frames - self.last_report_frames
        write_fps = dframes / dt if dt > 0 else 0.0
        self.last_report_t = now
        self.last_report_frames = self.frames
        meas = f"{self.fps_measured:.1f}" if self.fps_measured else "?"
        self.get_logger().info(
            f"segment {self.seg_index} : {self.frames} images ecrites "
            f"({self.cam_count} recues) | cadence camera={meas} img/s, "
            f"ecriture={write_fps:.1f} img/s")

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
