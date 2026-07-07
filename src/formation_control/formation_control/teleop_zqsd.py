#!/usr/bin/env python3
"""
teleop_zqsd.py — Teleop clavier ZQSD robuste pour TurtleBot3.

  Z : avancer        S : reculer
  Q : tourner gauche D : tourner droite
  A / E : avancer en tournant (gauche / droite)
  ESPACE : stop
  + / - : vitesse
  X : quitter (stop + sortie propre)

Particularites :
  - Auto-detection du topic : on lui donne un nom de robot (ex: tortuga1),
    il trouve TOUS les topics cmd_vel de ce robot (namespace simple OU double)
    et publie sur tous -> marche quel que soit l'etat du namespace.
  - Publication continue a 10 Hz de la derniere commande (maintien du
    mouvement fluide + le robot s'arrete des qu'on quitte).

Usage :
  ros2 run formation_control teleop_zqsd tortuga1
  ros2 run formation_control teleop_zqsd /tortuga1/cmd_vel   (topic explicite)
"""

import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
+--------------------------------------+
|         PILOTAGE  Z Q S D            |
|                                      |
|        Z : avancer                   |
|   Q  S  D : gauche / recul / droite  |
|      A/E : avance + vire (g/d)       |
|   ESPACE : STOP    +/- : vitesse     |
|        X : quitter                   |
+--------------------------------------+
"""

MOVES = {
    'z': (1.0,  0.0),
    's': (-1.0, 0.0),
    'q': (0.0,  1.0),
    'd': (0.0, -1.0),
    'a': (1.0,  0.7),
    'e': (1.0, -0.7),
}


class TeleopZQSD(Node):
    def __init__(self):
        super().__init__('teleop_zqsd')
        self.pubs = []
        self.lin = 0.15
        self.ang = 0.8
        self.cur = (0.0, 0.0)   # commande courante (facteurs)

    def setup_topics(self, target):
        """target = nom de robot ('tortuga1') ou topic explicite ('/x/cmd_vel')."""
        topics = []
        if target.startswith('/'):
            topics = [target]
        else:
            # Laisse le graphe ROS se peupler puis cherche les cmd_vel du robot
            deadline = time.time() + 3.0
            found = set()
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
                for name, types in self.get_topic_names_and_types():
                    if name.endswith('cmd_vel') and f'/{target}/' in name + '/':
                        if 'geometry_msgs/msg/Twist' in types:
                            found.add(name)
                if found:
                    break
            topics = sorted(found) if found else [f'/{target}/cmd_vel']
        for t in topics:
            self.pubs.append(self.create_publisher(Twist, t, 10))
        return topics

    def publish_current(self):
        t = Twist()
        t.linear.x = self.cur[0] * self.lin
        t.angular.z = self.cur[1] * self.ang
        for p in self.pubs:
            p.publish(t)

    def stop(self):
        self.cur = (0.0, 0.0)
        self.publish_current()


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if r else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = TeleopZQSD()

    argv = rclpy.utilities.remove_ros_args(sys.argv)
    target = argv[1] if len(argv) > 1 else 'tortuga1'
    topics = node.setup_topics(target)

    print(HELP)
    print(f"Robot cible : {target}")
    print("Topics utilises :")
    for t in topics:
        print(f"  -> {t}")
    print(f"Vitesses : lin={node.lin:.2f} m/s  ang={node.ang:.2f} rad/s")
    print("(maintien de la derniere commande ; ESPACE pour stopper)\n")

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            if key == 'x' or key == '\x03':      # x ou Ctrl-C
                break
            elif key == ' ':
                node.cur = (0.0, 0.0)
            elif key in MOVES:
                node.cur = MOVES[key]
            elif key == '+':
                node.lin = min(0.26, node.lin + 0.02)
                node.ang = min(1.8, node.ang + 0.1)
                print(f"vitesse lin={node.lin:.2f} ang={node.ang:.2f}")
            elif key == '-':
                node.lin = max(0.02, node.lin - 0.02)
                node.ang = max(0.2, node.ang - 0.1)
                print(f"vitesse lin={node.lin:.2f} ang={node.ang:.2f}")
            node.publish_current()               # ~10 Hz (timeout du get_key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop termine, robot stoppe.")


if __name__ == "__main__":
    main()
