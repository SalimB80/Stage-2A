#!/usr/bin/env python3
"""
teleop_zqsd.py — robust ZQSD keyboard teleop for TurtleBot3.

  Z : forward         S : backward
  Q : turn left       D : turn right
  A / E : forward while turning (left / right)
  SPACE : stop
  + / - : speed
  X : quit (stop + clean exit)

Notable points:
  - Topic auto-detection: give it a robot name (e.g. tortuga1) and it finds
    ALL the cmd_vel topics of that robot (single OR double namespace) and
    publishes on every one of them -> works whatever the namespace state is.
  - The last command is republished continuously at 10 Hz (keeps the motion
    smooth, and the robot stops as soon as we quit).

Usage:
  ros2 run formation_control teleop_zqsd tortuga1
  ros2 run formation_control teleop_zqsd /tortuga1/cmd_vel   (explicit topic)
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
        self.cur = (0.0, 0.0)   # current command (unit factors)

    def setup_topics(self, target):
        """target = robot name ('tortuga1') or explicit topic ('/x/cmd_vel')."""
        topics = []
        if target.startswith('/'):
            topics = [target]
        else:
            # Let the ROS graph populate, then look for the robot's cmd_vel
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
            if key == 'x' or key == '\x03':      # x or Ctrl-C
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
            node.publish_current()               # ~10 Hz (get_key timeout)
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
