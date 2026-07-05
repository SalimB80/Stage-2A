#!/usr/bin/env python3
"""
tracker_node.py — Robot autonome SANS leader.

Cherche une couleur (jaune) dans la piece, puis la suit a distance constante.
Fusion camera (direction) + lidar (distance).

Machine a etats, AVOID prioritaire (repris de l'approche Gemini) :
  AVOID   : obstacle proche NON aligne avec la couleur -> recule + tourne.
  ARRIVED : obstacle proche ALIGNE avec la couleur = la cible -> s'arrete,
            se cale a la distance voulue (ne FONCE PAS dedans).
  TRACK   : couleur vue, chemin libre -> avance vers elle, se centre.
  COAST   : couleur perdue depuis peu -> garde la derniere direction (probleme 2).
  SEARCH  : rien en vue -> explore (rotation + petits pas).

Lidar (probleme 1) : distance = point le plus proche dans un cone autour de la
direction visee -> mesure le ROBOT, pas le mur derriere.
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")

        # Couleur cible HSV (JAUNE par defaut) -> A CALIBRER
        self.declare_parameter("hsv_low",  [20, 80, 80])
        self.declare_parameter("hsv_high", [35, 255, 255])

        self.declare_parameter("target_distance", 0.6)
        self.declare_parameter("min_area", 800)
        self.declare_parameter("camera_hfov_deg", 62.0)

        self.declare_parameter("v_search", 0.10)
        self.declare_parameter("w_search", 0.6)
        self.declare_parameter("v_max", 0.15)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("k_lin", 0.6)
        self.declare_parameter("k_ang", 1.8)

        # Distances de securite
        self.declare_parameter("obstacle_dist", 0.30)   # obstacle si + proche
        self.declare_parameter("arrive_dist", 0.35)     # on se cale ici sur la cible
        self.declare_parameter("lidar_cone_deg", 8.0)   # cone lecture distance cible
        self.declare_parameter("align_tol_deg", 20.0)   # tolerance "obstacle=cible"
        self.declare_parameter("coast_time", 1.0)

        self.declare_parameter("v_avoid", 0.08)  # vitesse de recul en AVOID

        self.bridge = CvBridge()
        self.img_w = 640

        self.color_seen = False
        self.color_cx = 320
        self.color_area = 0
        self.target_angle = 0.0
        self.last_target_angle = 0.0
        self.target_distance = 99.0     # distance dans la direction de la couleur
        self.nearest_dist = 99.0        # obstacle le plus proche (toutes directions front)
        self.nearest_angle = 0.0        # sa direction

        self.state = "SEARCH"
        self.lost_since = None
        self.search_dir = random.choice([-1.0, 1.0])
        self.search_switch = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Image, "camera/image_raw", self.camera_cb, 5)
        self.create_subscription(LaserScan, "scan", self.lidar_cb, 5)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Tracker demarre (autonome, AVOID prioritaire)")

    # ---------- CAMERA ----------
    def camera_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge: {e}")
            return
        h, w = frame.shape[:2]
        self.img_w = w
        low = np.array(self.get_parameter("hsv_low").value, dtype=np.uint8)
        high = np.array(self.get_parameter("hsv_high").value, dtype=np.uint8)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        area = int(cv2.countNonZero(mask))
        self.color_area = area
        if area >= self.get_parameter("min_area").value:
            M = cv2.moments(mask)
            self.color_cx = int(M["m10"] / M["m00"])
            fov = math.radians(self.get_parameter("camera_hfov_deg").value)
            self.target_angle = -((self.color_cx - w/2.0)/(w/2.0))*(fov/2.0)
            self.last_target_angle = self.target_angle
            self.color_seen = True
        else:
            self.color_seen = False

    # ---------- LIDAR ----------
    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = 99.0
        n = len(ranges)

        def idx_of(angle):
            return max(0, min(n-1, int((angle - msg.angle_min)/msg.angle_increment)))

        # (Probleme 1) distance de la CIBLE : min dans un cone autour de la couleur
        angle = self.target_angle if self.color_seen else self.last_target_angle
        c = idx_of(angle)
        half = int(math.radians(
            self.get_parameter("lidar_cone_deg").value)/msg.angle_increment)
        win = ranges[max(0, c-half):min(n, c+half+1)]
        valid = win[(win > 0.10) & (win < 5.0)]
        self.target_distance = float(np.min(valid)) if len(valid) else 99.0

        # Obstacle le plus proche dans un large secteur frontal (+/-60 deg)
        fhalf = int(math.radians(60.0)/msg.angle_increment)
        lo = max(0, c*0 + idx_of(0.0) - fhalf)
        hi = min(n, idx_of(0.0) + fhalf + 1)
        front = ranges[lo:hi]
        fvalid_mask = (front > 0.10) & (front < 5.0)
        if np.any(fvalid_mask):
            local = np.argmin(np.where(fvalid_mask, front, 99.0))
            self.nearest_dist = float(front[local])
            self.nearest_angle = msg.angle_min + (lo + local)*msg.angle_increment
            # normalise dans [-pi, pi]
            self.nearest_angle = math.atan2(math.sin(self.nearest_angle),
                                            math.cos(self.nearest_angle))
        else:
            self.nearest_dist = 99.0
            self.nearest_angle = 0.0

    # ---------- CONTROLE ----------
    def control_loop(self):
        t = Twist()
        obst = self.get_parameter("obstacle_dist").value
        align_tol = math.radians(self.get_parameter("align_tol_deg").value)

        obstacle_near = self.nearest_dist < obst
        # L'obstacle proche est-il la CIBLE ? (couleur vue + memes directions)
        aligned = (self.color_seen and
                   abs(self.nearest_angle - self.target_angle) < align_tol)

        # 1) AVOID prioritaire : obstacle proche qui n'est PAS la cible
        if obstacle_near and not aligned:
            self.state = "AVOID"
            t.linear.x = -self.get_parameter("v_avoid").value
            # tourne a l'oppose de l'obstacle
            t.angular.z = self.get_parameter("w_max").value * \
                (-1.0 if self.nearest_angle > 0 else 1.0)
            self.cmd_pub.publish(t)
            return

        # 2) ARRIVED : obstacle proche ALIGNE = la cible -> on s'arrete dessus
        if obstacle_near and aligned:
            self.state = "ARRIVED"
            # on ne fonce pas : translation nulle, on se centre juste
            t.linear.x = 0.0
            t.angular.z = self._clamp(
                self.get_parameter("k_ang").value * self.target_angle,
                self.get_parameter("w_max").value)
            self.cmd_pub.publish(t)
            return

        # 3) Couleur vue, chemin libre -> TRACK
        if self.color_seen:
            self.state = "TRACK"
            self.lost_since = None
            self.cmd_pub.publish(self._track())
            return

        # 4) Couleur perdue -> COAST puis SEARCH
        now = self.get_clock().now()
        if self.lost_since is None:
            self.lost_since = now
        elapsed = (now - self.lost_since).nanoseconds/1e9
        if elapsed < self.get_parameter("coast_time").value \
           and self.target_distance < 5.0:
            self.state = "COAST"
            self.cmd_pub.publish(self._coast())
        else:
            self.state = "SEARCH"
            self.cmd_pub.publish(self._search())

    # ---------- COMPORTEMENTS ----------
    def _track(self):
        t = Twist()
        des = self.get_parameter("target_distance").value
        arrive = self.get_parameter("arrive_dist").value
        e_dist = self.target_distance - des

        # Si on est deja a/sous la distance d'arrivee : ne pas avancer
        if self.target_distance <= arrive:
            t.linear.x = 0.0
        else:
            t.linear.x = self._clamp(self.get_parameter("k_lin").value * e_dist,
                                     self.get_parameter("v_max").value)
        t.angular.z = self._clamp(self.get_parameter("k_ang").value * self.target_angle,
                                  self.get_parameter("w_max").value)
        if self.target_distance > 4.0:      # mesure lointaine peu sure -> prudence
            t.linear.x = min(t.linear.x, self.get_parameter("v_search").value)
        return t

    def _coast(self):
        t = Twist()
        t.linear.x = self.get_parameter("v_search").value * 0.5
        t.angular.z = self._clamp(
            self.get_parameter("k_ang").value * self.last_target_angle,
            self.get_parameter("w_max").value)
        return t

    def _search(self):
        t = Twist()
        now = self.get_clock().now()
        if (now - self.search_switch).nanoseconds/1e9 > 4.0:
            self.search_dir = random.choice([-1.0, 1.0])
            self.search_switch = now
        t.angular.z = self.get_parameter("w_search").value * self.search_dir
        t.linear.x = self.get_parameter("v_search").value * 0.5
        return t

    def _clamp(self, v, lim):
        return max(-lim, min(lim, v))


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
