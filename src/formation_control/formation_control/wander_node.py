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
        self.declare_parameter("critical_dist", 0.16)
        self.declare_parameter("obstacle_dist", 0.45)   # ralentit / prepare virage
        self.declare_parameter("safety_dist", 0.16)     # MESURE : sous 0.16 m
        #   collision en rotation. On ne descend jamais sous cette limite.
        self.declare_parameter("pause_time", 1.5)       # s d'arret quand obstacle
        self.declare_parameter("wait_clear_time", 2.0)  # s : si toujours bloque
        #   apres la pause, on considere l'obstacle fixe -> on change de cap.
        self.declare_parameter("front_deg", 25.0)       # demi-secteur frontal
        self.declare_parameter("heading_change_s", 5.0) # cap aleatoire toutes les ~Ns

        self.scan = None
        self.state = "FORWARD"
        self.odom_speed = 0.0
        self.stuck_since = None
        self.escape_until = None
        self.pause_since = None      # debut de la pause "attends que ca degage"
        self.turn_dir = 1.0
        self.next_heading_change = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.odom_cb, 5)
        self.create_timer(0.1, self.loop)
        self.get_logger().info("Wander demarre (lidar seul)")

    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        v = msg.twist.twist.linear
        self.odom_speed = math.sqrt(v.x*v.x + v.y*v.y)

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
        d_front = self.sector_min(self.scan, 0.0, front_half)
        d_left = self.sector_min(self.scan, math.radians(45), math.radians(30))
        d_right = self.sector_min(self.scan, math.radians(-45), math.radians(30))

        obst = self.get_parameter("obstacle_dist").value
        crit = self.get_parameter("critical_dist").value
        v = self.get_parameter("v_forward").value
        w = self.get_parameter("w_turn").value
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

        # --- SECURITE 0.16 en 3 temps ---
        # 1) obstacle proche -> on s'ARRETE (jamais sous 0.16 m).
        # 2) on attend 1-2 s : si l'objet s'ecarte, on repart tout droit.
        # 3) s'il est toujours la (fixe), on change de direction.
        blocked = d_front < obst
        if blocked:
            if self.pause_since is None:
                self.pause_since = now
            waited = (now - self.pause_since).nanoseconds / 1e9

            if waited < pause_t:
                # temps 1 : arret complet (on ne s'approche pas plus)
                self.state = "PAUSE"
                # si vraiment trop pres, petit recul minimal pour garder 0.16
                if d_front < safety + 0.03:
                    t.linear.x = -0.05
                self.cmd_pub.publish(t)
                return
            elif waited < pause_t + wait_t:
                # temps 2 : on attend encore un peu, immobile
                self.state = "WAIT"
                self.cmd_pub.publish(t)   # arret
                return
            else:
                # temps 3 : obstacle fixe -> on tourne vers le plus degage
                self.state = "REROUTE"
                self.turn_dir = 1.0 if d_left > d_right else -1.0
                t.angular.z = w * self.turn_dir
                # si degage devant apres avoir tourne, on relachera au prochain tick
                if d_front > obst * 1.2:
                    self.pause_since = None
                    self.state = "FORWARD"
                self.cmd_pub.publish(t)
                return
        else:
            # voie libre : on annule toute pause en cours
            self.pause_since = None

        # --- FORWARD normal + errance aleatoire ---
        self.state = "FORWARD"
        t.linear.x = v
        if now >= self.next_heading_change:
            self.turn_dir = random.choice([-1.0, 1.0])
            t.angular.z = random.uniform(0.2, 0.6) * self.turn_dir
            delay = self.get_parameter("heading_change_s").value
            self.next_heading_change = now + rclpy.duration.Duration(
                seconds=random.uniform(0.6 * delay, 1.6 * delay))
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
