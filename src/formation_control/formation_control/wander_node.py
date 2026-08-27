#!/usr/bin/env python3
"""
wander_node.py — random wandering on the lidar alone (for dataset collection).

The robot drives forward, changes heading at random, and avoids every obstacle
(walls, furniture, OTHER ROBOTS) using the lidar only. The camera is left
entirely free for recording.

States: FORWARD -> (obstacle) TURN -> FORWARD; BACKUP when too close.
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
        self.declare_parameter("critical_dist", 0.22)
        self.declare_parameter("obstacle_dist", 0.50)   # stop threshold ahead
        self.declare_parameter("safety_dist", 0.26)     # back off ONLY if closer
        #   the lidar sees the nearest point of the OTHER robot, but MY chassis
        #   sticks out in front of my lidar AND so does its own -> at 0.16 m of
        #   lidar range the two bodies are nearly touching. Two ~16 cm bodies +
        #   robots in motion (closing speed) -> we keep a 0.32 m lidar minimum.
        #   We never go below that limit.
        self.declare_parameter("pause_time", 1.0)       # 1 s stop: "wait, is it
        #   moving?"; if so we resume, otherwise we change direction.
        self.declare_parameter("front_deg", 25.0)       # narrow front half-sector
        self.declare_parameter("avoid_deg", 45.0)       # WIDE half-cone (body-aware)
        #   The robot is a 16x16 cm cube (half-diagonal ~0.11 m). While turning,
        #   its CORNERS sweep a circle: we therefore require a whole wide cone to
        #   be clear before resuming, otherwise a corner catches the obstacle.
        self.declare_parameter("robot_radius", 0.12)    # BUBBLE = CIRCUMSCRIBED
        #   circle of the 16x16 cm square: radius = half-diagonal ~0.113 m (plus
        #   a small margin). This radius is what covers the CORNERS; an 8 cm
        #   circle would miss them and let the bodies touch. It acts as the
        #   OMNIDIRECTIONAL avoidance floor (wide front cone + side sectors).
        # --- Continuous wandering (meander): instead of a single jerk every 5 s,
        # we apply a random angular velocity THAT LASTS and is renewed often ->
        # a winding trajectory, steady exploration.
        self.declare_parameter("wander_w_max", 0.35)    # meander amplitude (gentle)
        self.declare_parameter("wander_min_s", 3.5)     # heading held for a while
        self.declare_parameter("wander_max_s", 7.0)     # -> smooth stroll
        #   Headings held longer -> the robot drives longer in the same direction
        #   (straighter path, fewer heading changes).

        self.scan = None
        self.state = "FORWARD"
        self.odom_speed = 0.0
        # Message counters: terminal diagnostics (see debug_log).
        self.scan_count = 0
        self.odom_count = 0
        self.d_front = 99.0
        self.d_wide = 99.0
        self.d_left = 99.0
        self.d_right = 99.0
        self.stuck_since = None
        self.escape_until = None
        self.pause_since = None      # start of the "wait until it clears" pause
        self.turn_dir = 1.0
        self.avoid_dir = 0.0         # locked go-around direction (0 = free)
        self.wander_w = 0.0          # current random rotation (meander)
        self.next_wander = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Sensor QoS (BEST_EFFORT): mandatory for the Gazebo lidar, otherwise
        # no scan is received and the robot stays still.
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
        # Human-readable state in the terminal. If scan stays at 0 -> silent
        # sensor or QoS mismatch (the node waits for the scan and never moves).
        scan_ok = "OK" if self.scan_count else "!! AUCUN (QoS/topic ?)"
        odom_ok = "OK" if self.odom_count else "!! AUCUN (QoS/topic ?)"
        self.get_logger().info(
            f"[{self.state}] scan={self.scan_count}({scan_ok}) "
            f"odom={self.odom_count}({odom_ok}) vitesse={self.odom_speed:.3f} "
            f"d_front={self.d_front:.2f} d_wide={self.d_wide:.2f} "
            f"d_left={self.d_left:.2f} d_right={self.d_right:.2f}")

    def sector_min(self, msg, center_rad, half_rad):
        # Handles [0, 2pi] lidars: normalised angle + circular window.
        # Guard: an empty scan or one without angular step (angle_increment=0)
        # -> division by zero -> return "nothing seen" rather than crashing.
        if len(msg.ranges) == 0 or msg.angle_increment == 0.0:
            return 99.0
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
        d_wide = self.sector_min(self.scan, 0.0, avoid_half)   # WIDE body-aware cone
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

        # ANTI-STALL: we command forward motion but the odometry stays still
        # (a table leg invisible to the lidar...) -> back off + turn for 1.5 s.
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

        # --- body-aware avoidance (BUBBLE), in two stages ---
        # The robot is a 16x16 cm SQUARE: its bubble is the CIRCUMSCRIBED circle
        # (radius = robot_radius ~0.113 m). We watch the WHOLE front (wide cone
        # ±avoid_deg) AND the side sectors, not just the narrow ±front_deg front
        # cone: a robot arriving AT AN ANGLE was invisible to d_front and got
        # caught on a corner (hence "it turns, drives on and bumps again").
        #   breach = an obstacle enters the bubble (radius + small margin) from
        #            ANY front/side direction -> immediate danger.
        # 1) obstacle -> HARD STOP for 1 s. 2) still there -> PURE ROTATION until
        #    the bubble is genuinely clear, then a new heading.
        breach = min(d_front, d_wide, d_left, d_right) < radius + 0.05
        blocked = (d_wide < obst) or breach
        if blocked:
            if self.pause_since is None:
                self.pause_since = now
                self.avoid_dir = 1.0 if d_left > d_right else -1.0  # choisi 1 fois
            waited = (now - self.pause_since).nanoseconds / 1e9
            too_close = (d_wide < safety) or breach     # bubble pierced -> back off

            if waited < pause_t:
                # stage 1: full stop, we observe (hoping it clears)
                self.state = "PAUSE"
                if too_close:
                    t.linear.x = -0.06
                self.cmd_pub.publish(t)
                return

            # stage 2: go around by PURE ROTATION (reverse only if the bubble
            # is pierced), direction locked -> a clean trajectory.
            self.state = "REROUTE"
            t.angular.z = w * self.avoid_dir
            if too_close:                       # danger: back off to clear
                t.linear.x = -0.05
            # only resume once the wide cone AND the bubble are clear (avoids
            # setting off while the other robot is still on the front flank).
            if d_wide > obst * 1.25 and not breach:
                self.pause_since = None
                self.avoid_dir = 0.0
                self.next_wander = now          # draw a new heading right away
                self.state = "FORWARD"
            self.cmd_pub.publish(t)
            return
        else:
            # clear path: cancel the pause and the go-around lock
            self.pause_since = None
            self.avoid_dir = 0.0

        # --- FORWARD + CONTINUOUS WANDERING (meander) ---
        # A random rotation THAT LASTS (renewed every 1-2.5 s): the robot
        # snakes instead of going straight. We slow down slightly when an
        # obstacle appears in the wide cone, to anticipate the turn.
        self.state = "FORWARD"
        if now >= self.next_wander:
            wmax = self.get_parameter("wander_w_max").value
            self.wander_w = random.uniform(-wmax, wmax)
            self.next_wander = now + rclpy.duration.Duration(
                seconds=random.uniform(self.get_parameter("wander_min_s").value,
                                       self.get_parameter("wander_max_s").value))
        t.linear.x = v * (0.8 if d_wide < obst else 1.0)   # full speed unless close
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
