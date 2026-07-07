#!/usr/bin/env python3
"""
lidar_monitor.py — moniteur lidar temps reel dans le terminal.

Repond a UNE question : le lidar voit-il les obstacles ?
Affiche la distance dans 8 secteurs autour du robot + alerte obstacle.
Gere les lidars en [0, 2pi] comme en [-pi, pi].

Usage : python3 lidar_monitor.py tortuga3
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

# Secteurs : nom -> angle central (deg, 0=avant, +=gauche), demi-largeur deg
SECTORS = [
    ("AVANT      ",   0, 15),
    ("av-gauche  ",  45, 20),
    ("GAUCHE     ",  90, 20),
    ("ar-gauche  ", 135, 20),
    ("ARRIERE    ", 180, 20),
    ("ar-droit   ", 225, 20),
    ("DROIT      ", 270, 20),
    ("av-droit   ", 315, 20),
]
ALERT = 0.40   # m : seuil d'alerte obstacle


class LidarMonitor(Node):
    def __init__(self, ns):
        super().__init__("lidar_monitor")
        self.msg = None
        topic = f"/{ns}/scan" if not ns.startswith("/") else ns
        self.create_subscription(LaserScan, topic, self.cb,
                                 qos_profile_sensor_data)
        self.topic = topic
        self.create_timer(0.2, self.render)
        print(f"\nEcoute {topic} ... (Ctrl-C pour quitter)\n")

    def cb(self, msg):
        self.msg = msg

    def sector_min(self, center_deg, half_deg):
        m = self.msg
        r = np.array(m.ranges)
        r[np.isinf(r) | np.isnan(r)] = 99.0
        n = len(r)
        two_pi = 2 * math.pi
        a = math.radians(center_deg)
        while a < m.angle_min:
            a += two_pi
        while a >= m.angle_min + two_pi:
            a -= two_pi
        c = int((a - m.angle_min) / m.angle_increment) % n
        h = max(1, int(math.radians(half_deg) / m.angle_increment))
        idxs = np.arange(c - h, c + h + 1) % n
        w = r[idxs]
        valid = w[(w > 0.06) & (w < 90.0)]
        return float(np.min(valid)) if len(valid) else 99.0

    def render(self):
        if self.msg is None:
            print("  ... aucune donnee lidar recue pour l'instant ...")
            return
        # efface l'ecran (retour propre a chaque rafraichissement)
        print("\033[2J\033[H", end="")
        print(f"  LIDAR  {self.topic}   (alerte < {ALERT:.2f} m)\n")
        n_pts = len(self.msg.ranges)
        print(f"  {n_pts} points | angle_min={math.degrees(self.msg.angle_min):.0f}"
              f" deg  angle_max={math.degrees(self.msg.angle_max):.0f} deg\n")
        alert_front = False
        for name, ang, half in SECTORS:
            d = self.sector_min(ang, half)
            if d >= 90:
                bar, txt = "".ljust(20, "."), "  ---  "
            else:
                fill = max(0, min(20, int((2.0 - d) / 2.0 * 20)))
                bar = ("#" * fill).ljust(20, ".")
                txt = f"{d:5.2f}m"
            flag = ""
            if d < ALERT:
                flag = "  <<< OBSTACLE"
                if "AVANT" in name or "av-" in name:
                    alert_front = True
            print(f"  {name} |{bar}| {txt}{flag}")
        print()
        if alert_front:
            print("  >>> OBSTACLE DEVANT — le robot DEVRAIT eviter <<<")
        else:
            print("  voie avant degagee")


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else "tortuga3"
    rclpy.init()
    node = LidarMonitor(ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
