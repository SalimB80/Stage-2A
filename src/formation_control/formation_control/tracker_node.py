#!/usr/bin/env python3
"""
tracker_node.py v4 — autonomous colour following, LIDAR RELAY for the CASCADE.

Multi-colour cascade: each robot tracks the helmet of the robot in front of
it. tortuga1 = YELLOW helmet (the human-driven leader), tortuga2 (RED) follows
the yellow one, tortuga3 (GREEN) follows the red one, tortuga4 (BLUE) follows
the green one.

Key parameters:
  target_color    : jaune|rouge|vert|bleu|cyan (English aliases accepted:
                    yellow|red|green|blue) | 'custom'
  desired_bearing : angle (deg) at which the target should appear. 0 = column;
                    +30/-30 = flattened-V formations (line, triangle).
  target_distance : distance to hold (m). DEFAULT 0.32 (32 cm). Tunable per
                    robot, at runtime:
                      ros2 param set /tortugaX/tracker target_distance 0.40

>>> CALIBRATION: after a hsv_tuner.py session in the lab, copy the measured
>>> thresholds into the COLORS dict below (the single place to edit).

v4 (LIDAR RELAY) — the key requirement: the chain must NOT break when vision
loses the colour for a fraction of a second. Principle:
  1. the COLOUR identifies WHO to follow (the right helmet) and gives the
     direction;
  2. as soon as a helmet is seen, a lock is ARMED on that direction;
  3. if the colour disappears, the LIDAR TAKES OVER: it tracks the small blob
     (~6 cm across = a TurtleBot) nearest to the last known direction, and
     keeps following at target_distance;
  4. we only fall back to SEARCH (spin in place) after having lost BOTH the
     colour AND the lidar blob for lidar_lost_time seconds.
Outcome: a robot that "blinks" no longer wanders off, it stays hooked to the
robot ahead through the lidar while waiting to see the colour again.

States: AVOID (highest priority) > ARRIVED > TRACK (colour) > LIDAR (relay) >
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
# COLOUR PRESETS (OpenCV HSV: H 0-179, S 0-255, V 0-255)
# TO BE REPLACED by the values calibrated with hsv_tuner.py under the lab
# lighting. RED straddles H=0/179 -> TWO ranges combined.
# ---------------------------------------------------------------------------
COLORS = {
    "jaune": {"ranges": [([20, 80, 80],  [35, 255, 255])]},
    "cyan":  {"ranges": [([85, 80, 80],  [100, 255, 255])]},
    "rouge": {"ranges": [([0, 100, 80],  [8, 255, 255]),
                         ([172, 100, 80], [179, 255, 255])]},
    "vert":  {"ranges": [([40, 70, 60],  [85, 255, 255])]},
    "bleu":  {"ranges": [([100, 120, 60], [130, 255, 255])]},
}
# English aliases -> internal names (the GUI may send either one).
COLOR_ALIASES = {"yellow": "jaune", "red": "rouge", "green": "vert",
                 "blue": "bleu", "cyan": "cyan"}


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")

        self.declare_parameter("target_color", "jaune")
        self.declare_parameter("hsv_low",  [0, 0, 0])      # if target_color=custom
        self.declare_parameter("hsv_high", [0, 0, 0])
        self.declare_parameter("target_distance", 0.32)     # DEFAULT 32 cm
        self.declare_parameter("desired_bearing", 0.0)      # degrees
        self.declare_parameter("min_area", 800)
        self.declare_parameter("area_near", 12000)
        # area_near: helmet area (px^2) beyond which we are CLOSE to the
        # target -> stop advancing EVEN IF the lidar claims it is far away.
        # A camera-side brake, independent of the lidar.
        self.declare_parameter("camera_hfov_deg", 62.0)
        self.declare_parameter("v_search", 0.10)
        self.declare_parameter("w_search", 0.6)
        self.declare_parameter("v_max", 0.15)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("k_lin", 0.6)
        self.declare_parameter("k_ang", 1.8)
        self.declare_parameter("obstacle_dist", 0.45)  # ALIGNED with wander
        #   (0.50): a non-target obstacle triggers avoidance from 0.45 m.
        self.declare_parameter("safety_dist", 0.26)    # ALIGNED with wander:
        #   two 16 cm bodies + chassis overhanging the lidar -> we NEVER go
        #   below 0.26 m of lidar range (back off if closer).
        self.declare_parameter("arrive_dist", 0.32)    # approach stop, aligned
        #   on target_distance by default; recompute if the distance changes.
        self.declare_parameter("lidar_cone_deg", 8.0)
        self.declare_parameter("align_tol_deg", 20.0)
        self.declare_parameter("coast_time", 0.4)
        self.declare_parameter("v_avoid", 0.08)
        self.declare_parameter("area_when_near", 5000)
        # area_when_near: if an obstacle is close AND aligned with the
        # colour but the helmet is SMALL in the image, it is not the target
        # -> it is an obstacle, so avoid it.
        self.declare_parameter("stuck_time", 1.0)
        self.declare_parameter("escape_time", 1.3)
        self.declare_parameter("search_spin_only", True)

        # ---- LIDAR RELAY (v4) ---------------------------------------------
        self.declare_parameter("track_cone_deg", 35.0)
        #   half-cone searched for the blob around the last seen direction.
        self.declare_parameter("max_track_dist", 2.0)
        #   beyond 2 m the blob is no longer trusted (another robot/a wall).
        self.declare_parameter("lidar_lost_time", 3.0)
        #   time WITHOUT colour NOR blob before falling back to search.
        self.declare_parameter("blob_depth", 0.15)
        #   radial tolerance used to aggregate the points of one blob (m).
        self.declare_parameter("blob_max_width", 0.35)
        #   max physical width of a blob accepted as a robot (m). Wider than
        #   that means a wall/furniture -> ignored.

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

        # tracking lock (colour first, then lidar relay)
        self.lock_active = False
        self.lidar_lock_ok = False
        self.lidar_lock_angle = 0.0
        self.lidar_lock_dist = 99.0

        # Received-message counters: terminal diagnostics (QoS mismatch).
        self.cam_count = 0
        self.scan_count = 0
        self.odom_count = 0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Sensor QoS (BEST_EFFORT): a RELIABLE subscriber receives NOTHING.
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
        """Normalised colour name (accepts the GUI's English aliases)."""
        col = self.get_parameter("target_color").value
        return COLOR_ALIASES.get(col, col)

    # ---------- colour mask (handles the double-range red) ----------
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
            self.lock_active = True          # colour seen -> (re)arm the lock
        else:
            self.color_seen = False

    def odom_cb(self, msg):
        self.odom_count += 1
        v = msg.twist.twist.linear
        self.odom_speed = math.sqrt(v.x*v.x + v.y*v.y)

    # ---------- LIDAR ----------
    # NB: some lidars publish over [0, 2pi]. Camera angles (negative on the
    # right) are normalised into the scan frame, and the index windows WRAP
    # AROUND (modulo n) because 359 deg and 1 deg are neighbours.
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

        # target distance (brake): nearest point in a narrow cone around the
        # colour/lock direction.
        angle = self.target_angle if self.color_seen else self.last_target_angle
        c = idx_of(angle)
        half = int(math.radians(
            self.get_parameter("lidar_cone_deg").value)/msg.angle_increment)
        idxs = np.arange(c - half, c + half + 1) % n
        win = ranges[idxs]
        valid = win[(win > 0.10) & (win < 5.0)]
        self.target_distance = float(np.min(valid)) if len(valid) else 99.0

        # front sector (avoidance)
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

        # LIDAR RELAY: look for the nearest "robot" blob in a wide cone
        # around the last locked direction.
        self._update_lidar_lock(ranges, msg, idx_of)

    def _update_lidar_lock(self, ranges, msg, idx_of):
        """Detect the nearest blob (~a robot) around last_target_angle and
        update lidar_lock_*. A blob = a group of contiguous points at similar
        range, whose physical width is compatible with a TurtleBot."""
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

        # nearest point of the cone = blob seed
        seed = int(np.argmin(np.where(valid_mask, win, 99.0)))
        seed_r = win[seed]

        # grow left/right while the range stays close (same object)
        lo = seed
        while lo - 1 >= 0 and valid_mask[lo - 1] and \
                abs(win[lo - 1] - win[lo]) < depth:
            lo -= 1
        hi = seed
        while hi + 1 < len(win) and valid_mask[hi + 1] and \
                abs(win[hi + 1] - win[hi]) < depth:
            hi += 1

        span = (hi - lo) * msg.angle_increment            # angular width
        width = span * seed_r                             # physical width (m)
        if width > wmax:
            # too wide for a robot (wall/furniture) -> not a valid blob
            self.lidar_lock_ok = False
            return

        mid = (lo + hi) // 2
        a = msg.angle_min + int(idxs[mid]) * msg.angle_increment
        self.lidar_lock_angle = math.atan2(math.sin(a), math.cos(a))
        self.lidar_lock_dist = float(seed_r)
        self.lidar_lock_ok = True

    # ---------- CONTROL ----------
    def control_loop(self):
        t = Twist()
        now = self.get_clock().now()

        # 0) ANTI-STALL (absolute priority).
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

        # 1) COLOUR SEEN -> visual following (the most reliable)
        if self.color_seen:
            self.state = "TRACK"
            self.lost_since = None
            cmd = self._track()
            self.cmd_forward = cmd.linear.x > 0.04
            self.cmd_pub.publish(cmd)
            return

        # 2) COLOUR LOST but LOCK STILL ACTIVE: the LIDAR takes over. As
        # long as a robot blob is seen in the cone we keep following its
        # direction -> the chain does not break.
        if self.lock_active and self.lidar_lock_ok:
            self.lost_since = None
            self.state = "LIDAR"
            # feed the last known direction from the blob: when the colour
            # comes back, the camera takes over again cleanly.
            self.last_target_angle = self.lidar_lock_angle
            cmd = self._track_lidar()
            self.cmd_forward = cmd.linear.x > 0.04
            self.cmd_pub.publish(cmd)
            return

        # 3) NEITHER colour NOR blob: coast for a while, then search in place.
        self.cmd_forward = False
        if self.lost_since is None:
            self.lost_since = now
        elapsed = (now - self.lost_since).nanoseconds/1e9
        if elapsed < self.get_parameter("coast_time").value \
           and self.target_distance < 5.0:
            self.state = "COAST"
            self.cmd_pub.publish(self._coast())
        elif elapsed < self.get_parameter("lidar_lost_time").value:
            # the blob was just lost: hold the direction without rushing,
            # hoping to re-acquire colour or lidar very quickly.
            self.state = "HOLD"
            self.cmd_pub.publish(self._coast())
        else:
            self.lock_active = False          # lock dropped -> search
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
        """Angular error against the desired bearing (flattened V)."""
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
            if too_close:               # too close: small safety back-off
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
        """Following guided ONLY by the lidar (colour lost). We aim at the
        blob: turn towards its angle and hold target_distance. More cautious
        than colour following (capped speed), because the lidar discriminates
        less well than a colour."""
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
            # more cautious advance than in colour mode (0.7x, capped at
            # v_search*1.5)
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
