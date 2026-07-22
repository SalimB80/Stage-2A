#!/usr/bin/env python3
"""
tracker_node.py v4 — suivi autonome d'une couleur, RELAIS LIDAR pour la CASCADE.

Cascade multi-couleurs : chaque robot traque le casque du robot qui le
precede. tortuga1 = casque JAUNE (leader pilote), tortuga2 (ROUGE) suit le
jaune, tortuga3 (VERT) suit le rouge, tortuga4 (BLEU) suit le vert.

Parametres cles :
  target_color    : jaune|rouge|vert|bleu|cyan (alias anglais acceptes :
                    yellow|red|green|blue) | 'custom'
  desired_bearing : angle (deg) ou la cible doit apparaitre. 0 = colonne ;
                    +30/-30 = formations en V aplati (ligne, triangle).
  target_distance : distance a tenir (m). DEFAUT 0.32 (32 cm). Reglable par
                    robot, a chaud :
                      ros2 param set /tortugaX/tracker target_distance 0.40

>>> CALIBRATION : apres passage de hsv_tuner.py au labo, reporter les seuils
>>> mesures dans le dict COLORS ci-dessous (un seul endroit a editer).

v4 (RELAIS LIDAR) — le point cle demande : la chaine ne doit PAS casser
quand la vision perd la couleur une fraction de seconde. Principe :
  1. la COULEUR sert a identifier QUI suivre (le bon casque) et donne la
     direction ;
  2. des qu'un casque est vu, on ARME un verrou (lock) sur cette direction ;
  3. si la couleur disparait, le LIDAR PREND LE RELAIS : il traque le petit
     blob (~6 cm de diametre = un TurtleBot) le plus proche autour de la
     derniere direction connue, et continue de suivre a target_distance ;
  4. on ne repasse en RECHERCHE (rotation sur place) qu'apres avoir perdu
     ET la couleur ET le blob lidar pendant lidar_lost_time secondes.
Resultat : un robot qui « cligne » ne s'egare plus, il reste accroche au
robot de devant via le lidar en attendant de revoir la couleur.

Etats : AVOID (prioritaire) > ARRIVED > TRACK (couleur) > LIDAR (relais) >
        COAST > SEARCH.
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
    "vert":  {"ranges": [([40, 70, 60],  [85, 255, 255])]},
    "bleu":  {"ranges": [([100, 120, 60], [130, 255, 255])]},
}
# Alias anglais -> noms internes (le GUI peut envoyer l'un ou l'autre).
COLOR_ALIASES = {"yellow": "jaune", "red": "rouge", "green": "vert",
                 "blue": "bleu", "cyan": "cyan"}


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")

        self.declare_parameter("target_color", "jaune")
        self.declare_parameter("hsv_low",  [0, 0, 0])      # si target_color=custom
        self.declare_parameter("hsv_high", [0, 0, 0])
        self.declare_parameter("target_distance", 0.32)     # DEFAUT 32 cm
        self.declare_parameter("desired_bearing", 0.0)      # degres
        self.declare_parameter("min_area", 800)
        self.declare_parameter("area_near", 12000)
        # area_near : surface (px^2) du casque au-dela de laquelle on est
        # PRES de la cible -> arret de l'avance MEME si le lidar pretend
        # qu'elle est loin. Frein camera independant du lidar.
        self.declare_parameter("camera_hfov_deg", 62.0)
        self.declare_parameter("v_search", 0.10)
        self.declare_parameter("w_search", 0.6)
        self.declare_parameter("v_max", 0.15)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("k_lin", 0.6)
        self.declare_parameter("k_ang", 1.8)
        self.declare_parameter("obstacle_dist", 0.45)  # ALIGNE wander (0.50) :
        #   un obstacle non-cible declenche l'evitement des 0.45 m.
        self.declare_parameter("safety_dist", 0.26)    # ALIGNE wander : deux
        #   carrosseries de 16 cm + chassis qui depassent du lidar -> on ne
        #   descend JAMAIS sous 0.26 m lidar (recul si plus pres).
        self.declare_parameter("arrive_dist", 0.32)    # arret d'approche cale
        #   sur target_distance par defaut ; recalcule si distance changee.
        self.declare_parameter("lidar_cone_deg", 8.0)
        self.declare_parameter("align_tol_deg", 20.0)
        self.declare_parameter("coast_time", 0.4)
        self.declare_parameter("v_avoid", 0.08)
        self.declare_parameter("area_when_near", 5000)
        # area_when_near : si un obstacle est proche ET aligne avec la
        # couleur mais que le casque est PETIT a l'image, ce n'est pas la
        # cible -> obstacle, on evite.
        self.declare_parameter("stuck_time", 1.0)
        self.declare_parameter("escape_time", 1.3)
        self.declare_parameter("search_spin_only", True)

        # ---- RELAIS LIDAR (v4) --------------------------------------------
        self.declare_parameter("track_cone_deg", 35.0)
        #   demi-cone de recherche du blob autour de la derniere direction vue.
        self.declare_parameter("max_track_dist", 2.0)
        #   au-dela de 2 m on ne fait plus confiance au blob (autre robot/mur).
        self.declare_parameter("lidar_lost_time", 3.0)
        #   duree SANS couleur NI blob avant de repasser en recherche.
        self.declare_parameter("blob_depth", 0.15)
        #   tolerance radiale pour agreger les points d'un meme blob (m).
        self.declare_parameter("blob_max_width", 0.35)
        #   largeur physique max d'un blob accepte comme robot (m). Au-dela
        #   c'est un mur/meuble -> ignore.

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

        # verrou de suivi (couleur puis relais lidar)
        self.lock_active = False
        self.lidar_lock_ok = False
        self.lidar_lock_angle = 0.0
        self.lidar_lock_dist = 99.0

        # Compteurs de messages recus : diagnostic terminal (mismatch QoS).
        self.cam_count = 0
        self.scan_count = 0
        self.odom_count = 0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # QoS capteurs (BEST_EFFORT) : un subscriber RELIABLE ne recoit RIEN.
        self.create_subscription(Image, "camera/image_raw", self.camera_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self.lidar_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.odom_cb,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.debug_log)

        col = self._color_name()
        bear = self.get_parameter("desired_bearing").value
        dist = self.get_parameter("target_distance").value
        self.get_logger().info(
            f"Tracker v4 (relais lidar): suit '{col}' a {dist:.2f} m, "
            f"bearing {bear:.0f} deg")

    def _color_name(self):
        """Nom de couleur normalise (accepte les alias anglais du GUI)."""
        col = self.get_parameter("target_color").value
        return COLOR_ALIASES.get(col, col)

    # ---------- masque couleur (gere le rouge double-plage) ----------
    def color_mask(self, hsv):
        col = self._color_name()
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
            self.lock_active = True          # couleur vue -> on (re)verrouille
        else:
            self.color_seen = False

    def odom_cb(self, msg):
        self.odom_count += 1
        v = msg.twist.twist.linear
        self.odom_speed = math.sqrt(v.x*v.x + v.y*v.y)

    # ---------- LIDAR ----------
    # NB : certains lidars publient sur [0, 2pi]. Les angles camera (negatifs
    # a droite) sont normalises dans le repere du scan, et les fenetres
    # d'indices BOUCLENT (modulo n) car 359 deg et 1 deg sont voisins.
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

        # distance cible (frein) : point le plus proche dans un cone etroit
        # autour de la direction couleur/verrou.
        angle = self.target_angle if self.color_seen else self.last_target_angle
        c = idx_of(angle)
        half = int(math.radians(
            self.get_parameter("lidar_cone_deg").value)/msg.angle_increment)
        idxs = np.arange(c - half, c + half + 1) % n
        win = ranges[idxs]
        valid = win[(win > 0.10) & (win < 5.0)]
        self.target_distance = float(np.min(valid)) if len(valid) else 99.0

        # secteur frontal (evitement)
        fhalf = int(math.radians(60.0)/msg.angle_increment)
        z = idx_of(0.0)
        fidxs = np.arange(z - fhalf, z + fhalf + 1) % n
        front = ranges[fidxs]
        fm = (front > 0.10) & (front < 5.0)
        if np.any(fm):
            local = int(np.argmin(np.where(fm, front, 99.0)))
            self.nearest_dist = float(front[local])
            a = msg.angle_min + int(fidxs[local]) * msg.angle_increment
            self.nearest_angle = math.atan2(math.sin(a), math.cos(a))
        else:
            self.nearest_dist, self.nearest_angle = 99.0, 0.0

        # RELAIS LIDAR : cherche le blob "robot" le plus proche dans un large
        # cone autour de la derniere direction verrouillee.
        self._update_lidar_lock(ranges, msg, idx_of)

    def _update_lidar_lock(self, ranges, msg, idx_of):
        """Detecte le blob (~robot) le plus proche autour de last_target_angle
        et met a jour lidar_lock_*. Un blob = groupe de points contigus a
        distance voisine, de largeur physique compatible avec un TurtleBot."""
        n = len(ranges)
        if not self.lock_active:
            self.lidar_lock_ok = False
            return
        center = self.last_target_angle
        half = int(math.radians(
            self.get_parameter("track_cone_deg").value)/msg.angle_increment)
        dmax = self.get_parameter("max_track_dist").value
        depth = self.get_parameter("blob_depth").value
        wmax = self.get_parameter("blob_max_width").value

        idxs = np.arange(idx_of(center) - half,
                         idx_of(center) + half + 1) % n
        win = ranges[idxs]
        valid_mask = (win > 0.10) & (win < dmax)
        if not np.any(valid_mask):
            self.lidar_lock_ok = False
            return

        # point le plus proche du cone = graine du blob
        seed = int(np.argmin(np.where(valid_mask, win, 99.0)))
        seed_r = win[seed]

        # etendre a gauche/droite tant que la distance reste proche (meme objet)
        lo = seed
        while lo - 1 >= 0 and valid_mask[lo - 1] and \
                abs(win[lo - 1] - win[lo]) < depth:
            lo -= 1
        hi = seed
        while hi + 1 < len(win) and valid_mask[hi + 1] and \
                abs(win[hi + 1] - win[hi]) < depth:
            hi += 1

        span = (hi - lo) * msg.angle_increment            # largeur angulaire
        width = span * seed_r                             # largeur physique (m)
        if width > wmax:
            # trop large pour un robot (mur/meuble) -> pas un blob valide
            self.lidar_lock_ok = False
            return

        mid = (lo + hi) // 2
        a = msg.angle_min + int(idxs[mid]) * msg.angle_increment
        self.lidar_lock_angle = math.atan2(math.sin(a), math.cos(a))
        self.lidar_lock_dist = float(seed_r)
        self.lidar_lock_ok = True

    # ---------- CONTROLE ----------
    def control_loop(self):
        t = Twist()
        now = self.get_clock().now()

        # 0) ANTI-BLOCAGE (priorite absolue).
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
            if self.nearest_dist <= self.get_parameter("safety_dist").value:
                t.linear.x = -0.05
            t.angular.z = self._clamp(
                self.get_parameter("k_ang").value * self._bearing_error(),
                self.get_parameter("w_max").value)
            self.cmd_pub.publish(t)
            return

        # 1) COULEUR VUE -> suivi visuel (le plus fiable)
        if self.color_seen:
            self.state = "TRACK"
            self.lost_since = None
            cmd = self._track()
            self.cmd_forward = cmd.linear.x > 0.04
            self.cmd_pub.publish(cmd)
            return

        # 2) COULEUR PERDUE mais VERROU ACTIF : le LIDAR prend le relais.
        # Tant qu'un blob robot est vu dans le cone, on continue de suivre
        # sa direction -> la chaine ne se casse pas.
        if self.lock_active and self.lidar_lock_ok:
            self.lost_since = None
            self.state = "LIDAR"
            # on realimente la derniere direction connue avec le blob : quand
            # la couleur reviendra, la camera reprendra proprement le relais.
            self.last_target_angle = self.lidar_lock_angle
            cmd = self._track_lidar()
            self.cmd_forward = cmd.linear.x > 0.04
            self.cmd_pub.publish(cmd)
            return

        # 3) NI couleur NI blob : on temporise, puis recherche sur place.
        self.cmd_forward = False
        if self.lost_since is None:
            self.lost_since = now
        elapsed = (now - self.lost_since).nanoseconds/1e9
        if elapsed < self.get_parameter("coast_time").value \
           and self.target_distance < 5.0:
            self.state = "COAST"
            self.cmd_pub.publish(self._coast())
        elif elapsed < self.get_parameter("lidar_lost_time").value:
            # on a perdu le blob a l'instant : on tient la direction sans
            # foncer, en esperant re-accrocher couleur ou lidar tres vite.
            self.state = "HOLD"
            self.cmd_pub.publish(self._coast())
        else:
            self.lock_active = False          # verrou abandonne -> recherche
            self.state = "SEARCH"
            self.cmd_pub.publish(self._search())

    def debug_log(self):
        cam_ok = "OK" if self.cam_count else "!! AUCUN (QoS/topic ?)"
        scan_ok = "OK" if self.scan_count else "!! AUCUN (QoS/topic ?)"
        odom_ok = "OK" if self.odom_count else "!! AUCUN (QoS/topic ?)"
        lk = "oui" if self.lidar_lock_ok else "non"
        self.get_logger().info(
            f"[{self.state}] vu={self.color_seen} area={self.color_area} "
            f"d_cible={self.target_distance:.2f} "
            f"lock_lidar={lk}@{math.degrees(self.lidar_lock_angle):+.0f}deg/"
            f"{self.lidar_lock_dist:.2f}m "
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

    def _track_lidar(self):
        """Suivi guide UNIQUEMENT par le lidar (couleur perdue). On vise le
        blob : on tourne vers son angle et on tient target_distance. Plus
        prudent que le suivi couleur (vitesse plafonnee), car le lidar est
        moins discriminant qu'une couleur."""
        t = Twist()
        des = self.get_parameter("target_distance").value
        d = self.lidar_lock_dist
        safety = self.get_parameter("safety_dist").value
        too_close = self.nearest_dist <= safety + 0.04
        e_dist = d - des
        if too_close:
            t.linear.x = -0.05
        elif d <= max(des, self.get_parameter("arrive_dist").value):
            t.linear.x = 0.0
        else:
            # avance plus prudente qu'en couleur (0.7x, plafonnee a v_search*1.5)
            t.linear.x = self._clamp(
                0.7 * self.get_parameter("k_lin").value * e_dist,
                min(self.get_parameter("v_max").value,
                    self.get_parameter("v_search").value * 1.5))
        des_bear = math.radians(self.get_parameter("desired_bearing").value)
        t.angular.z = self._clamp(
            self.get_parameter("k_ang").value * (self.lidar_lock_angle - des_bear),
            self.get_parameter("w_max").value)
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
