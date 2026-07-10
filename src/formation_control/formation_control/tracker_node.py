#!/usr/bin/env python3
"""
tracker_node.py v3 — suivi autonome d'une couleur, adapte a la CASCADE.

Cascade multi-couleurs : chaque robot traque le casque du robot qui le
precede. tortuga1 = casque JAUNE (leader pilote), tortuga2 (ROUGE) suit le
jaune, tortuga3 (CYAN) suit le rouge, etc.

Parametres cles :
  target_color    : 'jaune' | 'rouge' | 'cyan' | 'custom'
  desired_bearing : angle (deg) ou la cible doit apparaitre. 0 = colonne ;
                    +30/-30 = formations en V aplati (ligne, triangle).
  target_distance : distance a tenir (m).

>>> CALIBRATION : apres passage de hsv_tuner.py au labo, reporter les seuils
>>> mesures dans le dict COLORS ci-dessous (un seul endroit a editer).

Etats : AVOID (prioritaire) > ARRIVED > TRACK > COAST > SEARCH.
Lidar : distance cible = point le plus proche dans un cone de +/-8 deg
autour de la direction vue par la camera (mesure le robot, pas le mur).
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# ---------------------------------------------------------------------------
# PRESETS COULEURS (HSV OpenCV : H 0-179, S 0-255, V 0-255)
# A REMPLACER par les valeurs calibrees avec hsv_tuner.py sous l'eclairage
# du labo. Le ROUGE est a cheval sur H=0/179 -> DEUX plages combinees.
# ---------------------------------------------------------------------------
COLORS = {
    "jaune": {"ranges": [([20, 80, 80],  [35, 255, 255])]},
    "cyan":  {"ranges": [([85, 80, 80],  [100, 255, 255])]},
    "rouge": {"ranges": [([0, 100, 80],  [8, 255, 255]),
                         ([172, 100, 80], [179, 255, 255])]},
}


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")

        self.declare_parameter("target_color", "jaune")
        self.declare_parameter("hsv_low",  [0, 0, 0])      # si target_color=custom
        self.declare_parameter("hsv_high", [0, 0, 0])
        self.declare_parameter("target_distance", 0.6)
        self.declare_parameter("desired_bearing", 0.0)      # degres
        self.declare_parameter("min_area", 800)
        self.declare_parameter("area_near", 12000)
        # area_near : surface (px^2) du casque au-dela de laquelle on est
        # PRES de la cible -> arret de l'avance MEME si le lidar pretend
        # qu'elle est loin. Frein camera independant du lidar, car le lidar
        # voit tres mal un TurtleBot (cible de 2-3 points seulement).
        self.declare_parameter("camera_hfov_deg", 62.0)
        self.declare_parameter("v_search", 0.10)
        self.declare_parameter("w_search", 0.6)
        self.declare_parameter("v_max", 0.15)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("k_lin", 0.6)
        self.declare_parameter("k_ang", 1.8)
        self.declare_parameter("obstacle_dist", 0.35)
        self.declare_parameter("safety_dist", 0.16)   # MESURE : collision en
        #   rotation sous 0.16 m. Le tracker ne s'approche jamais plus pres.
        self.declare_parameter("arrive_dist", 0.35)
        self.declare_parameter("lidar_cone_deg", 8.0)
        self.declare_parameter("align_tol_deg", 20.0)
        self.declare_parameter("coast_time", 1.0)
        self.declare_parameter("v_avoid", 0.08)
        self.declare_parameter("area_when_near", 5000)
        # area_when_near : si un obstacle est proche ET aligne avec la
        # couleur, mais que le casque est PETIT a l'image, ce n'est pas la
        # cible (un casque a 30 cm serait enorme) -> obstacle, on evite.
        self.declare_parameter("stuck_time", 1.0)
        self.declare_parameter("escape_time", 1.3)
        self.declare_parameter("search_spin_only", True)
        # search_spin_only=True : en recherche on TOURNE SANS AVANCER.
        # Plus sur en cascade : un robot qui a perdu sa cible ne s'eloigne
        # pas de la colonne, il balaie sur place.

        self.bridge = CvBridge()
        self.img_w = 640
        self.color_seen = False
        self.color_cx = 320
        self.color_area = 0
        self.target_angle = 0.0
        self.last_target_angle = 0.0
        self.target_distance = 99.0
        self.nearest_dist = 99.0
        self.nearest_angle = 0.0
        self.state = "SEARCH"
        self.lost_since = None
        self.odom_speed = 0.0
        self.cmd_forward = False
        self.stuck_since = None
        self.escape_until = None
        self.escape_dir = 1.0
        self.search_dir = random.choice([-1.0, 1.0])
        self.search_switch = self.get_clock().now()

        # Compteurs de messages recus : servent au diagnostic terminal.
        # Sans ca, impossible de savoir si un capteur ne publie pas ou si
        # c'est un MISMATCH QoS (publisher BEST_EFFORT vs subscriber RELIABLE).
        self.cam_count = 0
        self.scan_count = 0
        self.odom_count = 0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # QoS capteurs (BEST_EFFORT) : Gazebo/le lidar/la camera publient en
        # BEST_EFFORT. Un subscriber RELIABLE (defaut) ne recoit alors RIEN
        # -> le nœud reste bloque en SEARCH. On aligne donc sur sensor_data.
        self.create_subscription(Image, "camera/image_raw", self.camera_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self.lidar_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.odom_cb,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.debug_log)

        col = self.get_parameter("target_color").value
        bear = self.get_parameter("desired_bearing").value
        dist = self.get_parameter("target_distance").value
        self.get_logger().info(
            f"Tracker: suit '{col}' a {dist:.2f} m, bearing {bear:.0f} deg")

    # ---------- masque couleur (gere le rouge double-plage) ----------
    def color_mask(self, hsv):
        col = self.get_parameter("target_color").value
        if col == "custom":
            low = np.array(self.get_parameter("hsv_low").value, dtype=np.uint8)
            high = np.array(self.get_parameter("hsv_high").value, dtype=np.uint8)
            return cv2.inRange(hsv, low, high)
        spec = COLORS.get(col, COLORS["jaune"])
        mask = None
        for lo, hi in spec["ranges"]:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        return mask

    # ---------- CAMERA ----------
    def camera_cb(self, msg):
        self.cam_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge: {e}")
            return
        h, w = frame.shape[:2]
        self.img_w = w
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.color_mask(hsv)
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

    def odom_cb(self, msg):
        self.odom_count += 1
        v = msg.twist.twist.linear
        self.odom_speed = math.sqrt(v.x*v.x + v.y*v.y)

    # ---------- LIDAR ----------
    # NB : certains lidars (Coin-D4) publient sur [0, 2pi] et non [-pi, pi].
    # Les angles camera (negatifs a droite) sont donc NORMALISES dans le
    # repere du scan, et les fenetres d'indices BOUCLENT (modulo n) car
    # 359 deg et 1 deg sont voisins.
    def lidar_cb(self, msg):
        self.scan_count += 1
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = 99.0
        n = len(ranges)
        two_pi = 2.0 * math.pi

        def idx_of(a):
            while a < msg.angle_min:
                a += two_pi
            while a >= msg.angle_min + two_pi:
                a -= two_pi
            return int((a - msg.angle_min) / msg.angle_increment) % n

        angle = self.target_angle if self.color_seen else self.last_target_angle
        c = idx_of(angle)
        half = int(math.radians(
            self.get_parameter("lidar_cone_deg").value)/msg.angle_increment)
        idxs = np.arange(c - half, c + half + 1) % n      # fenetre circulaire
        win = ranges[idxs]
        valid = win[(win > 0.10) & (win < 5.0)]
        self.target_distance = float(np.min(valid)) if len(valid) else 99.0

        fhalf = int(math.radians(60.0)/msg.angle_increment)
        z = idx_of(0.0)
        fidxs = np.arange(z - fhalf, z + fhalf + 1) % n   # secteur frontal circulaire
        front = ranges[fidxs]
        m = (front > 0.10) & (front < 5.0)
        if np.any(m):
            local = int(np.argmin(np.where(m, front, 99.0)))
            self.nearest_dist = float(front[local])
            a = msg.angle_min + int(fidxs[local]) * msg.angle_increment
            self.nearest_angle = math.atan2(math.sin(a), math.cos(a))
        else:
            self.nearest_dist, self.nearest_angle = 99.0, 0.0

    # ---------- CONTROLE ----------
    def control_loop(self):
        t = Twist()
        now = self.get_clock().now()

        # 0) ANTI-BLOCAGE (priorite absolue). Si on commande d'avancer mais
        # que l'odometrie dit qu'on ne bouge pas -> coince (pied de table
        # trop fin pour le lidar, etc.) -> recul + rotation de degagement.
        if self.escape_until is not None:
            if now < self.escape_until:
                self.state = "STUCK"
                t.linear.x = -self.get_parameter("v_avoid").value
                t.angular.z = self.get_parameter("w_max").value * 0.7 \
                    * self.escape_dir
                self.cmd_pub.publish(t)
                return
            self.escape_until = None
            self.stuck_since = None

        if self.cmd_forward and self.odom_speed < 0.02:
            if self.stuck_since is None:
                self.stuck_since = now
            elif (now - self.stuck_since).nanoseconds/1e9 > \
                    self.get_parameter("stuck_time").value:
                self.get_logger().warn("COINCE : degagement (recul+rotation)")
                self.escape_dir = -1.0 if self.nearest_angle > 0 else 1.0
                self.escape_until = now + rclpy.duration.Duration(
                    seconds=self.get_parameter("escape_time").value)
                self.cmd_forward = False
                return
        else:
            self.stuck_since = None

        obst = self.get_parameter("obstacle_dist").value
        align_tol = math.radians(self.get_parameter("align_tol_deg").value)
        obstacle_near = self.nearest_dist < obst
        aligned = (self.color_seen and
                   abs(self.nearest_angle - self.target_angle) < align_tol)
        # Coherence camera/lidar : un objet proche "aligne" avec la couleur
        # n'est la cible que si le casque est GROS a l'image. Sinon c'est
        # un obstacle DEVANT la cible (pied de table...) -> on l'evite.
        if aligned and self.color_area < \
                self.get_parameter("area_when_near").value:
            aligned = False

        if obstacle_near and not aligned:
            self.state = "AVOID"
            self.cmd_forward = False
            t.linear.x = -self.get_parameter("v_avoid").value
            t.angular.z = self.get_parameter("w_max").value * \
                (-1.0 if self.nearest_angle > 0 else 1.0)
            self.cmd_pub.publish(t)
            return

        near_by_area = (self.color_seen and self.color_area >=
                        self.get_parameter("area_near").value)
        if (obstacle_near and aligned) or near_by_area:
            self.state = "ARRIVED"
            self.cmd_forward = False
            t.angular.z = self._clamp(
                self.get_parameter("k_ang").value * self._bearing_error(),
                self.get_parameter("w_max").value)
            self.cmd_pub.publish(t)
            return

        if self.color_seen:
            self.state = "TRACK"
            self.lost_since = None
            cmd = self._track()
            self.cmd_forward = cmd.linear.x > 0.04
            self.cmd_pub.publish(cmd)
            return

        self.cmd_forward = False
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

    def debug_log(self):
        # Diagnostic capteurs : cam/scan/odom = nb de messages recus depuis
        # le demarrage. Si l'un reste a 0, le capteur ne publie pas OU il y a
        # un mismatch QoS -> le nœud est "bloque" faute de donnees.
        cam_ok = "OK" if self.cam_count else "!! AUCUN (QoS/topic ?)"
        scan_ok = "OK" if self.scan_count else "!! AUCUN (QoS/topic ?)"
        odom_ok = "OK" if self.odom_count else "!! AUCUN (QoS/topic ?)"
        self.get_logger().info(
            f"[{self.state}] vu={self.color_seen} area={self.color_area} "
            f"d_cible={self.target_distance:.2f} "
            f"d_obst={self.nearest_dist:.2f}@{math.degrees(self.nearest_angle):+.0f}deg "
            f"| cam={self.cam_count}({cam_ok}) scan={self.scan_count}({scan_ok}) "
            f"odom={self.odom_count}({odom_ok}) vitesse={self.odom_speed:.3f}")

    def _bearing_error(self):
        """Erreur angulaire vs bearing desire (V aplati)."""
        des = math.radians(self.get_parameter("desired_bearing").value)
        return self.target_angle - des

    def _track(self):
        t = Twist()
        des = self.get_parameter("target_distance").value
        arrive = self.get_parameter("arrive_dist").value
        e_dist = self.target_distance - des
        near_by_area = self.color_area >= self.get_parameter("area_near").value
        too_close = self.nearest_dist <= self.get_parameter("safety_dist").value + 0.04
        if self.target_distance <= arrive or near_by_area or too_close:
            t.linear.x = 0.0
            if too_close:               # trop pres : petit recul de securite
                t.linear.x = -0.05
        else:
            t.linear.x = self._clamp(self.get_parameter("k_lin").value * e_dist,
                                     self.get_parameter("v_max").value)
        t.angular.z = self._clamp(
            self.get_parameter("k_ang").value * self._bearing_error(),
            self.get_parameter("w_max").value)
        if self.target_distance > 4.0:
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
        if not self.get_parameter("search_spin_only").value:
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
