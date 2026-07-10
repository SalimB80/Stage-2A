#!/usr/bin/env python3
"""
wander_node.py — errance aleatoire au lidar seul (pour la collecte dataset).

Le robot avance, change de cap aleatoirement, et evite tout obstacle
(murs, meubles, AUTRES ROBOTS) uniquement au lidar. La camera reste
entierement libre pour l'enregistrement.

Etats : FORWARD -> (obstacle) TURN -> FORWARD ; BACKUP si trop proche.
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class WanderNode(Node):
    def __init__(self):
        super().__init__("wander")
        self.declare_parameter("v_forward", 0.12)
        self.declare_parameter("w_turn", 0.8)
        self.declare_parameter("critical_dist", 0.28)
        self.declare_parameter("obstacle_dist", 0.60)   # ralentit / prepare virage
        self.declare_parameter("safety_dist", 0.32)     # DOUBLE de l'ancien 0.16 :
        #   le lidar voit le point le plus proche de l'AUTRE robot, mais MON
        #   chassis depasse devant mon lidar ET le sien depasse aussi -> a 0.16 m
        #   lidar, les carrosseries se touchent presque. Deux corps de ~16 cm +
        #   robots en mouvement (vitesse de rapprochement) -> on garde 0.32 m
        #   mini au lidar. On ne descend jamais sous cette limite.
        self.declare_parameter("pause_time", 2.0)       # s d'arret quand obstacle
        #   (on espere qu'il bouge avant de contourner).
        self.declare_parameter("wait_clear_time", 2.0)  # s : si toujours bloque
        #   apres la pause, on considere l'obstacle fixe -> on contourne.
        self.declare_parameter("front_deg", 25.0)       # demi-secteur frontal etroit
        self.declare_parameter("avoid_deg", 45.0)       # demi-cone LARGE (body-aware)
        #   Le robot est un cube 16x16 cm (demi-diagonale ~0.11 m). En rotation,
        #   ses COINS balaient un cercle : on exige donc que tout un cone large
        #   soit degage avant de repartir, sinon le coin accroche l'obstacle.
        self.declare_parameter("robot_radius", 0.18)    # demi-diagonale + marge
        #   pour DEUX corps (le mien + celui du robot croise) pendant un virage.
        # --- Errance continue (meandre) : au lieu d'un seul a-coup toutes les
        # 5 s, on applique une vitesse de rotation aleatoire QUI DURE et se
        # renouvelle souvent -> trajectoire sinueuse, exploration reguliere.
        self.declare_parameter("wander_w_max", 0.5)     # amplitude du meandre
        self.declare_parameter("wander_min_s", 3.0)     # duree mini d'un cap (x2.5)
        self.declare_parameter("wander_max_s", 6.5)     # duree maxi d'un cap (x2.6)
        #   Caps tenus plus longtemps -> le robot avance plus longtemps dans une
        #   meme direction (marche plus droite, moins de changements de cap).

        self.scan = None
        self.state = "FORWARD"
        self.odom_speed = 0.0
        # Compteurs de messages : diagnostic terminal (voir debug_log).
        self.scan_count = 0
        self.odom_count = 0
        self.d_front = 99.0
        self.d_wide = 99.0
        self.d_left = 99.0
        self.d_right = 99.0
        self.stuck_since = None
        self.escape_until = None
        self.pause_since = None      # debut de la pause "attends que ca degage"
        self.turn_dir = 1.0
        self.avoid_dir = 0.0         # sens de contournement verrouille (0 = libre)
        self.wander_w = 0.0          # rotation aleatoire courante (meandre)
        self.next_wander = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # QoS capteurs (BEST_EFFORT) : indispensable pour le lidar Gazebo,
        # sinon aucun scan recu et le robot reste immobile.
        self.create_subscription(LaserScan, "scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.odom_cb, qos_profile_sensor_data)
        self.create_timer(0.1, self.loop)
        self.create_timer(1.0, self.debug_log)
        self.get_logger().info("Wander demarre (lidar seul)")

    def scan_cb(self, msg):
        self.scan_count += 1
        self.scan = msg

    def odom_cb(self, msg):
        self.odom_count += 1
        v = msg.twist.twist.linear
        self.odom_speed = math.sqrt(v.x*v.x + v.y*v.y)

    def debug_log(self):
        # Etat lisible au terminal. Si scan reste a 0 -> capteur muet ou
        # mismatch QoS (le nœud attend le scan et ne bouge pas).
        scan_ok = "OK" if self.scan_count else "!! AUCUN (QoS/topic ?)"
        odom_ok = "OK" if self.odom_count else "!! AUCUN (QoS/topic ?)"
        self.get_logger().info(
            f"[{self.state}] scan={self.scan_count}({scan_ok}) "
            f"odom={self.odom_count}({odom_ok}) vitesse={self.odom_speed:.3f} "
            f"d_front={self.d_front:.2f} d_wide={self.d_wide:.2f} "
            f"d_left={self.d_left:.2f} d_right={self.d_right:.2f}")

    def sector_min(self, msg, center_rad, half_rad):
        # Gere les lidars en [0, 2pi] : angle normalise + fenetre circulaire.
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = 99.0
        n = len(ranges)
        two_pi = 2.0 * math.pi
        a = center_rad
        while a < msg.angle_min:
            a += two_pi
        while a >= msg.angle_min + two_pi:
            a -= two_pi
        c = int((a - msg.angle_min) / msg.angle_increment) % n
        h = int(half_rad / msg.angle_increment)
        idxs = np.arange(c - h, c + h + 1) % n
        w = ranges[idxs]
        valid = w[(w > 0.08) & (w < 8.0)]
        return float(np.min(valid)) if len(valid) else 99.0

    def loop(self):
        t = Twist()
        if self.scan is None:
            self.cmd_pub.publish(t)
            return

        front_half = math.radians(self.get_parameter("front_deg").value)
        avoid_half = math.radians(self.get_parameter("avoid_deg").value)
        d_front = self.sector_min(self.scan, 0.0, front_half)
        d_wide = self.sector_min(self.scan, 0.0, avoid_half)   # cone LARGE body-aware
        d_left = self.sector_min(self.scan, math.radians(45), math.radians(30))
        d_right = self.sector_min(self.scan, math.radians(-45), math.radians(30))
        self.d_front, self.d_wide = d_front, d_wide
        self.d_left, self.d_right = d_left, d_right

        obst = self.get_parameter("obstacle_dist").value
        crit = self.get_parameter("critical_dist").value
        v = self.get_parameter("v_forward").value
        w = self.get_parameter("w_turn").value
        radius = self.get_parameter("robot_radius").value
        now = self.get_clock().now()

        # ANTI-BLOCAGE : commande d'avancer mais odometrie immobile
        # (pied de table invisible au lidar...) -> recul + rotation 1.5 s.
        if self.escape_until is not None:
            if now < self.escape_until:
                t.linear.x = -0.08
                t.angular.z = w * self.turn_dir
                self.cmd_pub.publish(t)
                return
            self.escape_until = None
            self.stuck_since = None

        if self.state == "FORWARD" and self.odom_speed < 0.02:
            if self.stuck_since is None:
                self.stuck_since = now
            elif (now - self.stuck_since).nanoseconds/1e9 > 1.0:
                self.get_logger().warn("COINCE : degagement")
                self.turn_dir = 1.0 if d_left > d_right else -1.0
                self.escape_until = now + rclpy.duration.Duration(seconds=1.5)
                return
        else:
            self.stuck_since = None

        safety = self.get_parameter("safety_dist").value
        pause_t = self.get_parameter("pause_time").value
        wait_t = self.get_parameter("wait_clear_time").value
        clear = safety + radius        # marge minimale tenant compte du corps

        # --- EVITEMENT en 3 temps (securite 0.16, corps 16 cm) ---
        # 1) obstacle proche -> ARRET (2 s) en esperant qu'il bouge.
        # 2) on attend encore un peu, immobile.
        # 3) toujours la -> on CONTOURNE : rotation vers le cote le plus
        #    degage, MAINTENUE jusqu'a ce que le cone LARGE soit libre (=>
        #    l'obstacle passe sur le cote, camera ~parallele, coin degage).
        blocked = d_front < obst
        if blocked:
            if self.pause_since is None:
                self.pause_since = now
            waited = (now - self.pause_since).nanoseconds / 1e9

            if waited < pause_t:
                # temps 1 : arret complet (on ne s'approche pas plus)
                self.state = "PAUSE"
                if d_front < clear + 0.03:      # trop pres : petit recul
                    t.linear.x = -0.05
                self.cmd_pub.publish(t)
                return
            elif waited < pause_t + wait_t:
                # temps 2 : on attend encore, immobile
                self.state = "WAIT"
                self.cmd_pub.publish(t)
                return
            else:
                # temps 3 : contournement. On VERROUILLE le sens de rotation
                # (vers le plus degage) pour ne pas osciller, et on tourne
                # jusqu'a ce que le cone large soit franchement libre.
                self.state = "REROUTE"
                if self.avoid_dir == 0.0:
                    self.avoid_dir = 1.0 if d_left > d_right else -1.0
                # Le coin balaie un cercle de rayon ~radius : si un point est
                # trop pres (devant large OU du cote vers lequel on tourne),
                # on recule un peu en tournant pour ne pas frotter le coin.
                d_turnside = d_left if self.avoid_dir > 0 else d_right
                if min(d_wide, d_turnside) < clear:
                    t.linear.x = -0.05
                t.angular.z = w * self.avoid_dir
                # degage seulement quand le CONE LARGE est libre au-dela de
                # obstacle_dist -> l'obstacle est bien passe sur le cote.
                if d_wide > obst:
                    self.pause_since = None
                    self.avoid_dir = 0.0
                    self.state = "FORWARD"
                self.cmd_pub.publish(t)
                return
        else:
            # voie libre : on annule pause et verrou de contournement
            self.pause_since = None
            self.avoid_dir = 0.0

        # --- FORWARD + ERRANCE CONTINUE (meandre) ---
        # Rotation aleatoire QUI DURE (renouvelee toutes les 1-2.5 s) : le
        # robot serpente au lieu d'aller tout droit. On ralentit un peu si un
        # obstacle apparait dans le cone large, pour anticiper le virage.
        self.state = "FORWARD"
        if now >= self.next_wander:
            wmax = self.get_parameter("wander_w_max").value
            self.wander_w = random.uniform(-wmax, wmax)
            self.next_wander = now + rclpy.duration.Duration(
                seconds=random.uniform(self.get_parameter("wander_min_s").value,
                                       self.get_parameter("wander_max_s").value))
        t.linear.x = v * (0.6 if d_wide < obst * 1.5 else 1.0)
        t.angular.z = self.wander_w
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = WanderNode()
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
