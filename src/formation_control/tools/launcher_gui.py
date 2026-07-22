#!/usr/bin/env python3
"""
TurtleBot3 Control — v9 (clean English UI)

Two-layer architecture:
  1) BRINGUP : native TurtleBot3 bringup (motors + lidar) + camera.
  2) MODE    : wander | cascade | dataset, launched ON TOP of the bringup.
All commands use  ros2 run ... --ros-args -r __ns:=/tortugaX
(the validated method: single namespace, sensor QoS for the scan).

Inspection tabs: Log | Camera | Lidar | Topics | Dataset.
  - Camera : slider Raw / Color detection / Mask.
  - Lidar  : slider Points / Sectors / Map (accumulated).
"""

import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import subprocess
import threading
import time
import math
import os
import json

ROS_OK = True
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import (LaserScan, CompressedImage, BatteryState,
                                 CameraInfo)
    from nav_msgs.msg import Odometry
    import numpy as np
    import cv2
except Exception:
    ROS_OK = False

# ---------------- Fleet config ----------------
ROBOTS = {
    1: ("tortuga1", "192.168.0.201"),
    2: ("tortuga2", "192.168.0.202"),
    3: ("tortuga3", "192.168.0.203"),
    4: ("tortuga4", "192.168.0.204"),
}
HELMETS = {1: "yellow", 2: "red", 3: "green", 4: "blue"}
COLOR_NAMES = ["yellow", "red", "green", "blue", "cyan"]
FORMATION_BEARINGS = {
    "column": [0, 0, 0], "line": [30, -30, 30],
    "triangle": [25, -25, 0], "square": [20, -20, 0],
}
TARGET_DISTANCE = 0.32          # defaut cascade colonne : 32 cm
# Distance de suivi PAR ROBOT (m). Chaque suiveur peut avoir sa propre
# distance ; a defaut on retombe sur TARGET_DISTANCE (0.32). Reglable aussi
# a chaud sans relancer :
#   ros2 param set /tortugaX/tracker target_distance 0.40
FOLLOW_DIST = {1: 0.32, 2: 0.32, 3: 0.32, 4: 0.32}
SAFETY = 0.16
PW = "1234"
ROS_DOMAIN_ID = 30
DATASET_SH = "./src/formation_control/tools/dataset_tools.sh"
CONFIG_PATH = os.path.expanduser("~/.tb3_control_gui.json")

# HSV color presets for the camera "detection"/"mask" views (OpenCV HSV).
COLORS_HSV = {
    "yellow": [((20, 80, 80), (35, 255, 255))],
    "red":    [((0, 100, 80), (8, 255, 255)),
               ((172, 100, 80), (179, 255, 255))],
    "green":  [((40, 70, 60), (85, 255, 255))],
    "blue":   [((100, 120, 60), (130, 255, 255))],
    "cyan":   [((85, 80, 80), (100, 255, 255))],
}
DRAW_BGR = {"yellow": (0, 200, 220), "red": (40, 40, 220),
            "green": (60, 190, 60), "blue": (220, 120, 40),
            "cyan": (200, 190, 0)}

# ---------------- Palette (clean, one accent) ----------------
C_BG = "#ffffff"
C_PANE = "#f7f7f5"
C_PANE2 = "#eeeeec"
C_LINE = "#e3e3e0"
C_TXT = "#2b2a27"
C_SUB = "#6c6a66"
C_FAINT = "#9b9a97"
C_ACCENT = "#2f6fb3"
C_ACCENT_D = "#2a5f9a"
C_ACCENT_BG = "#eaf1f8"
C_OK = "#448361"
C_WARN = "#d9730d"
C_ERR = "#e03e3e"
C_ERRBG = "#fbecec"
HELMET_HEX = {"yellow": "#dfab01", "red": "#e03e3e", "green": "#3aa35a",
              "blue": "#3b6fd0", "cyan": "#2fa8b3"}

FONT = "Segoe UI"
MONO = "Cascadia Mono"


# ---------------- Environments & SSH ----------------
def robot_env():
    return (f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
            "export TURTLEBOT3_MODEL=burger; export LDS_MODEL=LDS-03; "
            "source /opt/ros/humble/setup.bash; "
            "source ~/turtlebot3_ws/install/setup.bash; "
            "source ~/formation_ws/install/setup.bash; ")


def pc_env():
    return (f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
            "source /opt/ros/humble/setup.bash; "
            "source ~/formation_ws/install/setup.bash; ")


def ssh_args(idx, cmd, timeout=4):
    user, ip = ROBOTS[idx]
    return ["sshpass", "-p", PW, "ssh", "-o", "StrictHostKeyChecking=no",
            "-o", f"ConnectTimeout={timeout}", f"{user}@{ip}", cmd]


def ssh_bg(idx, cmd):
    subprocess.Popen(ssh_args(idx, cmd),
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def open_terminal(bash_cmd, fallback):
    for att in (["wt.exe", "wsl.exe", "-e", "bash", "-lic", bash_cmd],
                ["cmd.exe", "/c", "start", "", "wsl.exe", "-e", "bash", "-lic",
                 bash_cmd]):
        try:
            subprocess.Popen(att, cwd="/mnt/c/",
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except FileNotFoundError:
            continue
    log(fallback, "warn")
    return False


# ---------------- ROS Hub (topics, scan, image, odom, battery) ----------------
class Hub:
    def __init__(self):
        self.frames = {}          # idx -> raw BGR image (as received)
        self.scans = {}           # idx -> LaserScan
        self.odom = {}            # idx -> (x, y, yaw)
        self.battery = {}         # idx -> voltage (V), for dataset auto-stop
        self.batt_pct = {}        # idx -> percentage (raw, may be 0-1 or 0-100)
        # last time (monotonic-ish, time.time()) a message actually arrived —
        # used for a TRUE liveness M/L/C (topic existence lingers in DDS after
        # the publisher dies, so it is NOT a reliable "alive" signal).
        self.t_scan = {}
        self.t_img = {}
        self.t_odom = {}
        self.t_cam = {}           # camera_info arrival (tiny) -> C liveness
        self.topics_by_robot = {i: set() for i in ROBOTS}
        self.topic_types = {}
        self.subbed = set()
        self.node = None
        self.started = False
        # Camera stream is heavy (55 fps): subscribe to ONE robot at a time,
        # and only while the Camera tab is open. Pulling all 4 at once saturates
        # the WiFi (SSH timeouts, M/L/C flicker, jerky control). want_cam is set
        # by the UI; the ROS thread reconciles the single active subscription.
        self.want_cam = None
        self.cam_idx = None
        self.cam_sub = None

    def start(self):
        if self.started or not ROS_OK:
            return
        self.started = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = RosNode("gui_hub")
            self.node.create_timer(2.0, self._discover)
            self.node.create_timer(0.4, self._reconcile_cam)
            rclpy.spin(self.node)
        except Exception as e:
            print(f"[hub ROS off] {e}")

    def _discover(self):
        if self.node is None:
            return
        seen = {i: set() for i in ROBOTS}
        for name, types in self.node.get_topic_names_and_types():
            self.topic_types[name] = types[0] if types else "?"
            for idx, (rn, _) in ROBOTS.items():
                if f"/{rn}/" not in name:
                    continue
                seen[idx].add(name)
                if name in self.subbed:
                    continue
                if name.endswith("/scan") and \
                        "sensor_msgs/msg/LaserScan" in types:
                    self.node.create_subscription(
                        LaserScan, name,
                        lambda m, i=idx: self._scan(i, m),
                        qos_profile_sensor_data)
                    self.subbed.add(name)
                elif name.endswith("/odom") and \
                        "nav_msgs/msg/Odometry" in types:
                    self.node.create_subscription(
                        Odometry, name,
                        lambda m, i=idx: self._odom(i, m), qos_profile_sensor_data)
                    self.subbed.add(name)
                elif name.endswith("/battery_state") and \
                        "sensor_msgs/msg/BatteryState" in types:
                    self.node.create_subscription(
                        BatteryState, name,
                        lambda m, i=idx: self._batt(i, m), 10)
                    self.subbed.add(name)
                elif name.endswith("/camera/camera_info") and \
                        "sensor_msgs/msg/CameraInfo" in types:
                    # tiny message at camera rate -> cheap C liveness without
                    # pulling the heavy image stream.
                    self.node.create_subscription(
                        CameraInfo, name,
                        lambda m, i=idx: self.t_cam.__setitem__(i, time.time()),
                        qos_profile_sensor_data)
                    self.subbed.add(name)
        self.topics_by_robot = seen

    def _reconcile_cam(self):
        """Keep exactly one compressed-camera subscription (the one the UI wants),
        or none. Runs in the ROS thread -> safe to create/destroy subs here."""
        if self.node is None or self.want_cam == self.cam_idx:
            return
        if self.cam_sub is not None:
            try:
                self.node.destroy_subscription(self.cam_sub)
            except Exception:
                pass
            self.cam_sub = None
        self.cam_idx = self.want_cam
        if self.cam_idx is not None:
            topic = f"/tortuga{self.cam_idx}/camera/image_raw/compressed"
            self.cam_sub = self.node.create_subscription(
                CompressedImage, topic,
                lambda m, i=self.cam_idx: self._img(i, m), qos_profile_sensor_data)

    def _scan(self, idx, msg):
        self.scans[idx] = msg
        self.t_scan[idx] = time.time()

    def _img(self, idx, msg):
        self.t_img[idx] = time.time()
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is not None:
                self.frames[idx] = bgr
        except Exception:
            pass

    def _odom(self, idx, msg):
        self.t_odom[idx] = time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom[idx] = (p.x, p.y, yaw)

    def _batt(self, idx, msg):
        self.battery[idx] = msg.voltage
        self.batt_pct[idx] = msg.percentage


# ---------------- Small helpers ----------------
def sector_min(msg, center_deg, half_deg):
    if not msg.ranges or msg.angle_increment == 0.0:
        return 99.0
    r = np.array(msg.ranges)
    r[np.isinf(r) | np.isnan(r)] = 99.0
    n = len(r)
    if n == 0:
        return 99.0
    two_pi = 2 * math.pi
    a = math.radians(center_deg)
    while a < msg.angle_min:
        a += two_pi
    while a >= msg.angle_min + two_pi:
        a -= two_pi
    c = int((a - msg.angle_min) / msg.angle_increment) % n
    h = max(1, int(math.radians(half_deg) / msg.angle_increment))
    idxs = np.arange(c - h, c + h + 1) % n
    w = r[idxs]
    valid = w[(w > 0.06) & (w < 90.0)]
    return float(np.min(valid)) if len(valid) else 99.0


def color_mask(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in COLORS_HSV.get(color, COLORS_HSV["yellow"]):
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def parse_num(s, default=0.0):
    try:
        return float(str(s).replace(",", "."))
    except Exception:
        return default


def to_photo(bgr):
    """np BGR image -> Tk PhotoImage (PPM). imencode('.ppm') writes correct RGB."""
    ok, buf = cv2.imencode(".ppm", bgr)
    return tk.PhotoImage(data=buf.tobytes()) if ok else None


# ---------------- Selection & chain ----------------
def present_indices():
    return sorted(i for i in ROBOTS if present_vars[i].get())


def targets():
    return present_indices() or list(ROBOTS)


def build_chain():
    ch = present_indices()
    if not ch:
        return []
    be = FORMATION_BEARINGS.get(form_var.get(), [0, 0, 0])
    out = [(ch[0], "leader", None, 0.0)]
    for k, rob in enumerate(ch[1:]):
        out.append((rob, "tracker", HELMETS[ch[k]], be[k] if k < len(be) else 0))
    return out


# ---------------- Log ----------------
def log(msg, level="info"):
    ts = time.strftime("%H:%M:%S")
    tag = {"info": "info", "ok": "ok", "warn": "warn", "err": "err"}.get(level, "info")
    journal.configure(state="normal")
    journal.insert("end", f"{ts}  ", "time")
    journal.insert("end", f"{msg}\n", tag)
    journal.see("end")
    journal.configure(state="disabled")
    status.set(msg)


def refresh_info(*_):
    m = mode_var.get()
    launch_btn.config(text={"cascade": "Start cascade",
                            "wander": "Start wander",
                            "dataset": "Start dataset"}[m])
    form_combo.configure(state="readonly" if m == "cascade" else "disabled")
    present = present_indices()
    if not present:
        info.set("Select one or more robots.")
        return
    if m == "cascade":
        ch = build_chain()
        parts = [f"t{ch[0][0]} · leader"] + \
                [f"t{r}→{c}" for r, _, c, b in ch[1:]]
        info.set("Chain:  " + "    ".join(parts))
    elif m == "wander":
        info.set("Autonomous wander (safety 0.32 m) — "
                 + ", ".join(f"t{i}" for i in present))
    else:
        info.set("Dataset video + lidar — " + ", ".join(f"t{i}" for i in present))
    try:
        update_formation()
    except Exception:
        pass


# ---------------- State (ping + disk) ----------------
def refresh_state():
    log("Refreshing state…")
    def work():
        res = {}
        for i, (_, ip) in ROBOTS.items():
            try:
                ok = subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    timeout=3).returncode == 0
            except Exception:
                ok = False
            free = "—"
            if ok:
                try:
                    r = subprocess.run(ssh_args(i, "df -h ~ | tail -1 | awk '{print $4}'",
                                                timeout=4),
                                       capture_output=True, text=True, timeout=8)
                    free = (r.stdout or "—").strip() or "—"
                except Exception:
                    free = "?"
            res[i] = (ok, free)
        root.after(0, lambda: apply_state(res))
    threading.Thread(target=work, daemon=True).start()


def apply_state(res):
    for i, (ok, free) in res.items():
        dots[i].configure(fg=C_OK if ok else C_ERR)
        space_lbl[i].config(text=free)
        p = batt_pct(i)
        if p is None:
            batt_lbl[i].config(text="—", fg=C_SUB)
        else:
            batt_lbl[i].config(
                text=f"{p:.0f}%",
                fg=C_ERR if p < 20 else (C_WARN if p < 40 else C_OK))
    log("State refreshed.", "ok")


# ---------------- Layer 1: BRINGUP ----------------
def bringup_start():
    present = present_indices()
    if not present:
        log("Select at least one robot.", "warn")
        return
    for i in present:
        ssh_bg(i, robot_env() +
               f"ros2 launch turtlebot3_bringup robot.launch.py "
               f"namespace:=tortuga{i}")
        ssh_bg(i, robot_env() +
               f"ros2 run camera_ros camera_node "
               f"--ros-args -r __ns:=/tortuga{i} -r __node:=camera "
               f"-p format:=BGR888 -p width:=640 -p height:=480 "
               # 640x480 @ 55 fps : 18181 us/image. No space in the array (a
               # space splits the argument -> ignored). The recorder stores the
               # native JPEG frames (no re-encode) so the Pi holds 55 fps.
               f"-p FrameDurationLimits:=[18181,18181] "
               f"-r ~/image_raw:=camera/image_raw")
    log("Bringup + camera started: " + ", ".join(f"t{i}" for i in present)
        + " (~15 s to boot).", "ok")


def bringup_stop():
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[r]obot.launch'; pkill -9 -f '[t]urtlebot3_ros'; "
                  "pkill -9 -f '[s]ingle_coin'; pkill -9 -f '[c]amera_node'; "
                  "pkill -9 -f '[r]obot_state'; pkill -9 -f '[d]iff_drive'; "
                  "pkill -9 -f '[l]d08'")
    log("Bringup stopped: " + ", ".join(f"t{i}" for i in present), "warn")


# ---------------- Layer 2: MODE ----------------
def behavior_start():
    present = present_indices()
    if not present:
        log("Select at least one robot.", "warn")
        return
    m = mode_var.get()
    form = form_var.get()
    chain = build_chain() if m == "cascade" else None
    seg_minutes = 5.0
    if m == "dataset":
        seg_minutes = parse_num(seg_var.get(), 5.0)
        if seg_minutes < 0.2:
            seg_minutes = 5.0
    launch_btn.config(state="disabled")
    log("Preparing… (stopping previous mode)")

    def work():
        # 1) Stop the previous mode and WAIT for the kills to finish.
        # Launching without waiting created a RACE between the async pkill and
        # the async ros2 run -> the pkill could land after the launch and kill
        # the just-started node. We kill blocking, then let DDS drop the nodes.
        procs = [subprocess.Popen(
                    ssh_args(i, "pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
                                "pkill -TERM -f '[r]ecorder'; "
                                "pkill -TERM -f '[b]ag record'", timeout=4),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                 for i in present]
        for p in procs:
            try:
                p.wait(timeout=8)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        time.sleep(0.8)

        # 2) Launch the chosen mode.
        if m == "cascade":
            for rob, role, color, bear in chain:
                if role == "leader":
                    root.after(0, lambda r=rob:
                               log(f"t{r} = leader (drive it with ZQSD)."))
                    continue
                # Single-range color -> push the tuned HSV as a custom filter
                # (so the HSV Tuner values actually apply). Multi-range (red)
                # -> use the named preset in tracker_node.
                ranges = COLORS_HSV.get(color, [])
                if len(ranges) == 1:
                    lo, hi = ranges[0]
                    cspec = (f"-p target_color:=custom "
                             f"-p hsv_low:=[{lo[0]},{lo[1]},{lo[2]}] "
                             f"-p hsv_high:=[{hi[0]},{hi[1]},{hi[2]}]")
                else:
                    cspec = f"-p target_color:={color}"
                # Lancement DETACHE (nohup + &) avec log sur le robot, comme
                # dataset_tools.sh. Sinon le ros2 run tourne en foreground de
                # la session SSH : si la connexion se ferme/hoquete (WiFi), le
                # tracker recoit SIGHUP et MEURT -> robot fige, sans trace.
                # Le log ~/tracker_tortugaX.log permet de diagnostiquer.
                tracker_cmd = (
                    f"ros2 run formation_control tracker "
                    f"--ros-args -r __ns:=/tortuga{rob} "
                    # float() OBLIGATOIRE : les bearings sont des int (0, 30…)
                    # mais le param est declare en DOUBLE cote tracker -> ROS2
                    # rejette un int et le nœud CRASHE au demarrage. On envoie
                    # donc '0.0' et non '0'. Idem target_distance (deja float).
                    f"{cspec} -p desired_bearing:={float(bear)} "
                    f"-p target_distance:={float(FOLLOW_DIST.get(rob, TARGET_DISTANCE))}")
                ssh_bg(rob, robot_env() +
                       f"nohup {tracker_cmd} "
                       f"> ~/tracker_tortuga{rob}.log 2>&1 &")
            root.after(0, lambda: log(f"Cascade started ({form}).", "ok"))
        else:
            for i in present:
                ssh_bg(i, robot_env() +
                       f"ros2 run formation_control wander "
                       f"--ros-args -r __ns:=/tortuga{i}")
                if m == "dataset":
                    # recorder writes camera JPEGs + scan.csv + odom.csv itself
                    # (timestamped) -> no rosbag needed, lighter on the Pi.
                    ssh_bg(i, robot_env() +
                           f"ros2 run formation_control recorder "
                           f"--ros-args -r __ns:=/tortuga{i} "
                           f"-p robot_name:=tortuga{i} "
                           f"-p segment_minutes:={seg_minutes}")
            msg = ("Dataset started (recording!): " if m == "dataset"
                   else "Wander started: ") + ", ".join(f"t{i}" for i in present)
            root.after(0, lambda msg=msg: log(msg, "ok"))
            if m == "dataset":
                root.after(0, start_guard)
        root.after(0, lambda: launch_btn.config(state="normal"))

    threading.Thread(target=work, daemon=True).start()


def behavior_stop(silent=False):
    stop_guard()
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
                  "pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'")
    if not silent:
        log("Mode stopped (bringup kept): "
            + ", ".join(f"t{i}" for i in present), "warn")


# ---------------- Dataset watchdog (auto-stop) ----------------
# During a dataset session, monitors ELAPSED time, battery VOLTAGE (via the ROS
# hub) and FREE storage (df over SSH). When a threshold is crossed, the mode is
# stopped CLEANLY (behavior_stop -> SIGTERM to recorder/rosbag, segments closed
# properly). Bringup stays up so the data can be pulled. Thresholds live in the
# "Dataset" inspection tab.
dataset_guard = {"active": False, "t0": 0.0}


def robot_free_gb(i):
    try:
        r = subprocess.run(ssh_args(i, "df -B1 ~ | tail -1 | awk '{print $4}'",
                                    timeout=4),
                           capture_output=True, text=True, timeout=8)
        b = (r.stdout or "").strip()
        return int(b) / 1e9 if b.isdigit() else None
    except Exception:
        return None


def start_guard():
    dataset_guard["active"] = True
    dataset_guard["t0"] = time.time()
    threading.Thread(target=guard_loop, daemon=True).start()
    log("Dataset watchdog active (duration · battery · storage).")


def stop_guard():
    dataset_guard["active"] = False


def guard_loop():
    while dataset_guard["active"]:
        reason = None
        dur = parse_num(dur_var.get())      # min ; 0 = unlimited
        batt = parse_num(batt_var.get())    # %  ; 0 = disabled
        disk = parse_num(disk_var.get())    # GB ; 0 = disabled
        targets_ = present_indices() or list(ROBOTS)
        if dur > 0 and (time.time() - dataset_guard["t0"]) >= dur * 60:
            reason = f"duration {dur:.0f} min reached"
        if reason is None and batt > 0:
            for i in targets_:
                p = batt_pct(i)
                if p is not None and p < batt:
                    reason = f"tortuga{i} battery low ({p:.0f}%)"
                    break
        if reason is None and disk > 0:
            for i in targets_:
                free = robot_free_gb(i)
                if free is not None and free < disk:
                    reason = f"tortuga{i} storage low ({free:.1f} GB)"
                    break
        if reason:
            dataset_guard["active"] = False
            tg = list(targets_)
            root.after(0, lambda r=reason, t=tg: _auto_stop(r, t))
            return
        time.sleep(15)


def _auto_stop(reason, tg):
    # Always close the recording cleanly (SIGTERM recorder/rosbag, stop wander).
    behavior_stop()
    # Only power OFF the robots if the user explicitly enabled it (off by
    # default -> no surprise fleet shutdown while testing).
    if shutdown_var.get():
        log(f"AUTO-STOP dataset: {reason} — closing files then SHUTTING DOWN.",
            "warn")
        def later():
            time.sleep(5)
            root.after(0, lambda: shutdown_robots(tg))
        threading.Thread(target=later, daemon=True).start()
    else:
        log(f"AUTO-STOP dataset: {reason} — recording stopped (bringup kept, "
            f"no shutdown).", "warn")


def shutdown_robots(idxs):
    for i in idxs:
        ssh_bg(i, f"echo {PW} | sudo -S shutdown -h now")
    log("Shutdown sent (poweroff): " + ", ".join(f"t{i}" for i in idxs), "warn")


def apply_formation():
    if mode_var.get() != "cascade":
        log("Formation only applies in cascade mode.", "warn")
        return
    for rob, _, _, bear in build_chain()[1:]:
        subprocess.Popen(["bash", "-c", pc_env() +
            f"ros2 param set /tortuga{rob}/tracker desired_bearing {bear}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Formation applied live ({form_var.get()}).", "ok")


def teleop_robot(idx):
    if open_terminal(pc_env() +
                     f"ros2 run formation_control teleop_zqsd tortuga{idx}",
                     f"Manual: teleop_zqsd tortuga{idx}"):
        log(f"ZQSD teleop tortuga{idx} (new window).")


def halt_robot(idx):
    # Publish a single zero Twist on /tortugaX/cmd_vel (instant motion halt).
    # NB: if wander/tracker is still running it keeps publishing cmd_vel and
    # will override this — use "Stop mode" to end the behavior for good.
    twist = ('"{linear: {x: 0.0, y: 0.0, z: 0.0}, '
             'angular: {x: 0.0, y: 0.0, z: 0.0}}"')
    cmd = (pc_env() + f"ros2 topic pub --once /tortuga{idx}/cmd_vel "
           f"geometry_msgs/msg/Twist {twist}")
    subprocess.Popen(["bash", "-c", cmd],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Halt sent to tortuga{idx}.", "warn")


def batt_pct(i):
    """Battery percentage for robot i, or None. Handles 0-1 and 0-100 drivers,
    with a voltage-based fallback for a 3S LiPo (~10.0 V empty, 12.6 V full)."""
    if hub is None:
        return None
    p = hub.batt_pct.get(i)
    if p is not None and p > 0:
        return p * 100.0 if p <= 1.0 else p
    v = hub.battery.get(i)
    if v is not None and v > 0.5:
        return max(0.0, min(100.0, (v - 10.0) / (12.6 - 10.0) * 100.0))
    return None


# ---------------- Full STOP & build ----------------
KILL_CMD = ("pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'; sleep 1; "
            "pkill -9 -f '[r]obot.launch'; pkill -9 -f '[t]urtlebot3_ros'; "
            "pkill -9 -f '[s]ingle_coin'; pkill -9 -f '[c]amera_node'; "
            "pkill -9 -f '[r]obot_state'; pkill -9 -f '[d]iff_drive'; "
            "pkill -9 -f '[l]d08'; pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
            "pkill -9 -f '[f]ollower'; pkill -9 -f -- '[-]-ros-args'; sleep 0.5; "
            "pkill -9 -f -- '[-]-ros-args'; true")
CHECK_CMD = "pgrep -fc -- '[-]-ros-args' || true"


def stop_everything():
    log("STOP — killing all nodes…", "warn")
    stop_btn.config(state="disabled")
    def work():
        procs = {i: subprocess.Popen(ssh_args(i, KILL_CMD, timeout=3),
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL) for i in ROBOTS}
        for p in procs.values():
            try:
                p.wait(timeout=12)
            except subprocess.TimeoutExpired:
                p.kill()
        rep = []
        for i in ROBOTS:
            try:
                r = subprocess.run(ssh_args(i, CHECK_CMD, timeout=3),
                                   capture_output=True, text=True, timeout=8)
                out = (r.stdout or "").strip().splitlines()
                n = out[-1] if out and out[-1].isdigit() else "0"
                rep.append(f"t{i} " + ("✓" if n == "0" else f"{n}!"))
            except Exception:
                rep.append(f"t{i} off")
        root.after(0, lambda: (stop_btn.config(state="normal"),
                               log("Stop done — " + "   ".join(rep),
                                   "ok" if all("!" not in x for x in rep) else "warn")))
    threading.Thread(target=work, daemon=True).start()


def build_deploy():
    idxs = " ".join(map(str, present_indices())) or "1 2 3 4"
    log(f"Build + deploy on {idxs}… (1–2 min)")
    def work():
        try:
            r = subprocess.run(
                ["bash", "-c",
                 "cd ~/formation_ws && colcon build --symlink-install && "
                 "source install/setup.bash && "
                 # 'bash <script>' (not './<script>') so a missing +x bit or a
                 # CRLF shebang can never block the deploy again.
                 f"bash ./src/formation_control/tools/deploy_build.sh {idxs}"],
                capture_output=True, text=True, timeout=240)
            if r.returncode == 0:
                root.after(0, lambda: log("Build + deploy done.", "ok"))
            else:
                out = (r.stdout + r.stderr)[-300:]
                root.after(0, lambda: log(f"Build failed: {out}", "err"))
        except subprocess.TimeoutExpired:
            root.after(0, lambda: log("Build: timeout (>4 min).", "err"))
        except Exception as e:
            root.after(0, lambda: log(f"Build: exception {e}", "err"))
    threading.Thread(target=work, daemon=True).start()


# ---------------- Data ----------------
def ds_pull():
    idx = " ".join(map(str, targets()))
    open_terminal(pc_env() + f"cd ~/formation_ws && bash {DATASET_SH} collect {idx}; "
                  "echo; echo '=== Done. Enter ==='; read",
                  "Manual: dataset_tools.sh collect")
    log(f"Pull started (robots {idx}).")


def ds_clean():
    idx = " ".join(map(str, targets()))
    if not messagebox.askyesno("Delete data",
            f"Delete ALL data (video + lidar) on {idx}?\n\n"
            "Pull first to be safe."):
        return
    for i in targets():
        ssh_bg(i, "rm -rf ~/dataset/*")
    log(f"Delete started (robots {idx}).", "warn")


# ---------------- Settings persistence ----------------
def _cfg_read():
    try:
        with open(CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def _cfg_write(d):
    try:
        with open(CONFIG_PATH, "w") as f:
            json.dump(d, f, indent=2)
        return True
    except Exception as e:
        log(f"Could not write config: {e}", "err")
        return False


def save_settings():
    d = _cfg_read()
    d.update({"duration": dur_var.get(), "segment": seg_var.get(),
              "battery": batt_var.get(), "storage": disk_var.get(),
              "helmets": {str(i): HELMETS[i] for i in ROBOTS}})
    if _cfg_write(d):
        log(f"Settings saved to {CONFIG_PATH}.", "ok")


def load_settings():
    d = _cfg_read()
    dur_var.set(d.get("duration", dur_var.get()))
    seg_var.set(d.get("segment", seg_var.get()))
    batt_var.set(d.get("battery", batt_var.get()))
    disk_var.set(d.get("storage", disk_var.get()))
    for i in ROBOTS:
        c = d.get("helmets", {}).get(str(i))
        if c in COLORS_HSV:
            HELMETS[i] = c


def save_hsv():
    d = _cfg_read()
    d["hsv"] = {k: [[list(a), list(b)] for a, b in v]
                for k, v in COLORS_HSV.items()}
    if _cfg_write(d):
        log("HSV presets saved.", "ok")


def load_hsv():
    d = _cfg_read()
    for k, ranges in d.get("hsv", {}).items():
        try:
            COLORS_HSV[k] = [(tuple(a), tuple(b)) for a, b in ranges]
        except Exception:
            pass


def load_helmets():
    d = _cfg_read()
    for i in ROBOTS:
        c = d.get("helmets", {}).get(str(i))
        if c in COLORS_HSV:
            HELMETS[i] = c


# ---------------- HSV tuner window ----------------
def open_hsv_tuner():
    win = tk.Toplevel(root)
    win.title("HSV Tuner")
    win.configure(bg=C_BG)
    win.geometry("560x560")

    top = tk.Frame(win, bg=C_BG)
    top.pack(fill="x", padx=14, pady=10)
    tk.Label(top, text="Preset", bg=C_BG, fg=C_SUB, font=(FONT, 9)).pack(side="left")
    preset = tk.StringVar(value=cam_color.get() if cam_color.get() in COLORS_HSV
                          else COLOR_NAMES[0])
    ttk.Combobox(top, textvariable=preset, values=COLOR_NAMES,
                 state="readonly", width=8).pack(side="left", padx=6)
    tk.Label(top, text="   Robot", bg=C_BG, fg=C_SUB,
             font=(FONT, 9)).pack(side="left")
    rob = tk.IntVar(value=cam_robot.get())
    for i in ROBOTS:
        tk.Radiobutton(top, text=f"t{i}", variable=rob, value=i, bg=C_BG,
                       fg=C_TXT, selectcolor="#fff", activebackground=C_BG,
                       font=(FONT, 9)).pack(side="left")

    names = ["H low", "S low", "V low", "H high", "S high", "V high"]
    maxes = [179, 255, 255, 179, 255, 255]
    svars = [tk.IntVar() for _ in range(6)]
    for n, mx, var in zip(names, maxes, svars):
        r = tk.Frame(win, bg=C_BG)
        r.pack(fill="x", padx=14)
        tk.Label(r, text=n, width=7, anchor="w", bg=C_BG, fg=C_TXT,
                 font=(FONT, 9)).pack(side="left")
        tk.Scale(r, from_=0, to=mx, orient="horizontal", variable=var,
                 length=380, bg=C_BG, fg=C_TXT, troughcolor=C_PANE2,
                 highlightthickness=0, sliderrelief="flat").pack(side="left")

    prev = tk.Frame(win, bg=C_BG)
    prev.pack(pady=8)
    c_img = tk.Canvas(prev, width=248, height=186, bg="#000",
                      highlightthickness=1, highlightbackground=C_LINE)
    c_img.grid(row=0, column=0, padx=4)
    c_mask = tk.Canvas(prev, width=248, height=186, bg="#000",
                       highlightthickness=1, highlightbackground=C_LINE)
    c_mask.grid(row=0, column=1, padx=4)
    val_lbl = tk.Label(win, text="", bg=C_BG, fg=C_SUB, font=(MONO, 9))
    val_lbl.pack()

    def cur_range():
        v = [x.get() for x in svars]
        return ((v[0], v[1], v[2]), (v[3], v[4], v[5]))

    def load_preset(*_):
        rng = COLORS_HSV.get(preset.get(), [((0, 0, 0), (0, 0, 0))])[0]
        (hl, sl, vl), (hh, sh, vh) = rng
        for var, val in zip(svars, [hl, sl, vl, hh, sh, vh]):
            var.set(int(val))

    preset.trace_add("write", load_preset)

    def do_save():
        COLORS_HSV[preset.get()] = [cur_range()]   # single range (per tuner)
        save_hsv()
        lo, hi = cur_range()
        log(f"HSV saved for {preset.get()}: low={list(lo)} high={list(hi)}", "ok")

    btns = tk.Frame(win, bg=C_BG)
    btns.pack(pady=8)
    tk.Button(btns, text="Apply + Save", command=do_save, bg=C_ACCENT,
              fg="white", bd=0, font=(FONT, 10, "bold"), cursor="hand2",
              activebackground=C_ACCENT_D, padx=14, pady=6).pack(side="left")
    tk.Label(win, text="Red wraps H=0/179 (two ranges); the tuner edits a single "
             "range — keep red's presets or edit tracker_node for the 2nd range.",
             bg=C_BG, fg=C_FAINT, font=(FONT, 8), wraplength=520).pack(padx=14)

    load_preset()

    def loop():
        if not win.winfo_exists():
            return
        lo, hi = cur_range()
        val_lbl.config(text=f"low={list(lo)}   high={list(hi)}")
        if hub is not None and ROS_OK:
            bgr = hub.frames.get(rob.get())
            if bgr is not None:
                small = cv2.resize(bgr, (248, 186))
                im = to_photo(small)
                if im:
                    c_img._img = im
                    c_img.delete("all")
                    c_img.create_image(124, 93, image=im)
                hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array(lo, np.uint8),
                                   np.array(hi, np.uint8))
                mk = to_photo(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
                if mk:
                    c_mask._img = mk
                    c_mask.delete("all")
                    c_mask.create_image(124, 93, image=mk)
        win.after(130, loop)

    loop()


# ============================================================
#                        INTERFACE
# ============================================================
root = tk.Tk()
root.title("TurtleBot3 Control")
root.geometry("1220x880")
root.configure(bg=C_BG)
root.minsize(1120, 820)

# Load persisted HSV presets + helmet colors before building the UI.
load_hsv()
load_helmets()

style = ttk.Style(root)
style.theme_use("clam")
style.configure("TNotebook", background=C_BG, borderwidth=0)
style.configure("TNotebook.Tab", background=C_PANE, foreground=C_SUB,
                padding=(16, 8), font=(FONT, 9), borderwidth=0)
style.map("TNotebook.Tab",
          background=[("selected", C_BG)], foreground=[("selected", C_TXT)])
style.configure("TCombobox", fieldbackground="#fff", background="#fff",
                foreground=C_TXT, arrowcolor=C_SUB, bordercolor=C_LINE)
style.configure("Accent.Horizontal.TScale", background=C_BG)


def hsep(parent):
    tk.Frame(parent, bg=C_LINE, height=1).pack(fill="x", pady=8)


def section(parent, text):
    tk.Label(parent, text=text, bg=C_BG, fg=C_FAINT,
             font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 5))


# ---- left column (control) + right column (inspection) ----
main = tk.Frame(root, bg=C_BG)
main.pack(fill="both", expand=True)

left = tk.Frame(main, bg=C_BG, width=440)
left.pack(side="left", fill="y", expand=False)
left.pack_propagate(False)

right = tk.Frame(main, bg=C_PANE)
right.pack(side="right", fill="both", expand=True)

# ================= LEFT COLUMN =================
head = tk.Frame(left, bg=C_BG)
head.pack(fill="x", padx=22, pady=(18, 2))
tk.Label(head, text="TurtleBot3 Control", bg=C_BG, fg=C_TXT,
         font=(FONT, 17, "bold")).pack(anchor="w")
tk.Label(head, text="Formations · Wander · Dataset", bg=C_BG, fg=C_FAINT,
         font=(FONT, 9)).pack(anchor="w")

# --- bottom block (pinned): data + STOP ---
bottom = tk.Frame(left, bg=C_BG)
bottom.pack(fill="x", side="bottom", padx=22, pady=(0, 16))
hsep(bottom)
dr = tk.Frame(bottom, bg=C_BG)
dr.pack(fill="x", pady=(0, 6))
tk.Button(dr, text="Pull data", command=ds_pull, bg=C_PANE, fg=C_TXT,
          bd=0, font=(FONT, 9), cursor="hand2", activebackground=C_PANE2,
          pady=6).pack(side="left", expand=True, fill="x", padx=(0, 3))
tk.Button(dr, text="Delete", command=ds_clean, bg=C_PANE, fg=C_ERR,
          bd=0, font=(FONT, 9), cursor="hand2", activebackground=C_ERRBG,
          pady=6).pack(side="left", expand=True, fill="x", padx=(3, 0))
stop_btn = tk.Button(bottom, text="STOP — kill everything", command=stop_everything,
                     bg=C_ERR, fg="white", bd=0, font=(FONT, 11, "bold"),
                     cursor="hand2", activebackground="#c43535", pady=9)
stop_btn.pack(fill="x")

# --- body (fills remaining space above the pinned bottom) ---
body = tk.Frame(left, bg=C_BG)
body.pack(fill="both", expand=True, padx=22, pady=10)

# Robots
section(body, "ROBOTS")
present_vars, dots, space_lbl, batt_lbl, state_dot, helmet_sw = \
    {}, {}, {}, {}, {}, {}
for i in ROBOTS:
    row = tk.Frame(body, bg=C_BG)
    row.pack(fill="x", pady=1)
    dots[i] = tk.Label(row, text="●", bg=C_BG, fg=C_FAINT, font=(FONT, 9))
    dots[i].pack(side="left")
    helmet_sw[i] = tk.Label(row, text="■", bg=C_BG,
                            fg=HELMET_HEX[HELMETS[i]], font=(FONT, 11))
    helmet_sw[i].pack(side="left", padx=(4, 2))
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_info)
    tk.Checkbutton(row, text=f"tortuga{i}", variable=v, bg=C_BG, fg=C_TXT,
                   selectcolor="#fff", activebackground=C_BG,
                   activeforeground=C_TXT, font=(FONT, 9),
                   anchor="w", width=9).pack(side="left")
    st = tk.Frame(row, bg=C_BG)
    st.pack(side="left", padx=4)
    state_dot[i] = {}
    for key in ("M", "L", "C"):
        lb = tk.Label(st, text=key, bg=C_PANE2, fg=C_FAINT,
                      font=(MONO, 8, "bold"), width=2, height=1)
        lb.pack(side="left", padx=1)
        state_dot[i][key] = lb
    space_lbl[i] = tk.Label(row, text="—", bg=C_BG, fg=C_SUB,
                            font=(MONO, 8), width=5, anchor="e")
    space_lbl[i].pack(side="left", padx=(4, 0))
    batt_lbl[i] = tk.Label(row, text="—", bg=C_BG, fg=C_SUB,
                           font=(MONO, 8), width=4, anchor="e")
    batt_lbl[i].pack(side="left", padx=(2, 0))
    tk.Button(row, text="ZQSD", command=lambda i=i: teleop_robot(i),
              bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 8), cursor="hand2",
              activebackground=C_PANE2, padx=5).pack(side="right")
    tk.Button(row, text="Halt", command=lambda i=i: halt_robot(i),
              bg=C_ERRBG, fg=C_ERR, bd=0, font=(FONT, 8), cursor="hand2",
              activebackground="#f6dede", padx=5).pack(side="right", padx=(0, 3))

tk.Button(body, text="Refresh  ·  ping + disk + battery", command=refresh_state,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, pady=5).pack(fill="x", pady=(8, 0))

hsep(body)

# 1 · Build
section(body, "1 · BUILD")
tk.Button(body, text="Build + Deploy", command=build_deploy,
          bg=C_ACCENT, fg="white", bd=0, font=(FONT, 10, "bold"),
          cursor="hand2", activebackground=C_ACCENT_D, pady=8).pack(fill="x")

hsep(body)

# 2 · Hardware
section(body, "2 · HARDWARE")
r1 = tk.Frame(body, bg=C_BG)
r1.pack(fill="x")
tk.Button(r1, text="Start robot", command=bringup_start,
          bg=C_ACCENT, fg="white", bd=0, font=(FONT, 10, "bold"),
          cursor="hand2", activebackground=C_ACCENT_D, pady=8).pack(
    side="left", expand=True, fill="x", padx=(0, 4))
tk.Button(r1, text="Stop", command=bringup_stop,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, padx=14).pack(side="left")

hsep(body)

# 3 · Mode
section(body, "3 · MODE")
mode_var = tk.StringVar(value="wander")
mode_var.trace_add("write", refresh_info)
for val, txt, desc in (
        ("wander", "Wander", "autonomous exploration, avoidance"),
        ("cascade", "Cascade", "driven leader + color followers"),
        ("dataset", "Dataset", "wander + video & lidar recording")):
    rr = tk.Frame(body, bg=C_BG)
    rr.pack(fill="x", pady=1)
    tk.Radiobutton(rr, variable=mode_var, value=val, bg=C_BG,
                   selectcolor="#fff", activebackground=C_BG).pack(side="left")
    tk.Label(rr, text=txt, bg=C_BG, fg=C_TXT,
             font=(FONT, 10, "bold")).pack(side="left")
    tk.Label(rr, text=" · " + desc, bg=C_BG, fg=C_FAINT,
             font=(FONT, 9)).pack(side="left")

fr = tk.Frame(body, bg=C_BG)
fr.pack(fill="x", pady=(6, 0))
tk.Label(fr, text="Formation", bg=C_BG, fg=C_SUB,
         font=(FONT, 9)).pack(side="left")
form_var = tk.StringVar(value="column")
form_var.trace_add("write", refresh_info)
form_combo = ttk.Combobox(fr, textvariable=form_var,
                          values=list(FORMATION_BEARINGS.keys()),
                          state="readonly", width=10, font=(FONT, 9))
form_combo.pack(side="left", padx=6)
tk.Button(fr, text="Apply live", command=apply_formation,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 8), cursor="hand2",
          activebackground=C_PANE2, padx=8).pack(side="left")

info = tk.StringVar(value="")
tk.Label(body, textvariable=info, bg=C_ACCENT_BG, fg=C_ACCENT,
         font=(FONT, 9), anchor="w", justify="left", wraplength=380,
         padx=10, pady=6).pack(fill="x", pady=(8, 0))

r2 = tk.Frame(body, bg=C_BG)
r2.pack(fill="x", pady=(6, 0))
launch_btn = tk.Button(r2, text="Start wander", command=behavior_start,
                       bg=C_OK, fg="white", bd=0, font=(FONT, 10, "bold"),
                       cursor="hand2", activebackground="#3a6f52", pady=8)
launch_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
tk.Button(r2, text="Stop mode", command=lambda: behavior_stop(),
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, padx=14).pack(side="left")

# ================= RIGHT COLUMN : INSPECTION =================
tk.Label(right, text="INSPECTION", bg=C_PANE, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", padx=18, pady=(16, 6))

nb = ttk.Notebook(right)
nb.pack(fill="both", expand=True, padx=14, pady=(0, 14))

# --- Tab: Log ---
tab_log = tk.Frame(nb, bg=C_BG)
nb.add(tab_log, text="   Log   ")
journal = tk.Text(tab_log, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9),
                  state="disabled", wrap="word", padx=12, pady=10)
journal.pack(fill="both", expand=True)
journal.tag_configure("time", foreground=C_FAINT)
journal.tag_configure("info", foreground=C_TXT)
journal.tag_configure("ok", foreground=C_OK)
journal.tag_configure("warn", foreground=C_WARN)
journal.tag_configure("err", foreground=C_ERR)

# --- Tab: Camera ---
CAM_W, CAM_H = 640, 480
tab_cam = tk.Frame(nb, bg=C_BG)
nb.add(tab_cam, text="   Camera   ")
cam_top = tk.Frame(tab_cam, bg=C_BG)
cam_top.pack(fill="x", padx=12, pady=(10, 4))
tk.Label(cam_top, text="Robot", bg=C_BG, fg=C_SUB, font=(FONT, 9)).pack(side="left")
cam_robot = tk.IntVar(value=1)
for i in ROBOTS:
    tk.Radiobutton(cam_top, text=f"t{i}", variable=cam_robot, value=i,
                   bg=C_BG, fg=C_TXT, selectcolor="#fff",
                   activebackground=C_BG, font=(FONT, 9)).pack(side="left")
tk.Label(cam_top, text="   Color", bg=C_BG, fg=C_SUB,
         font=(FONT, 9)).pack(side="left")
cam_color = tk.StringVar(value="yellow")
ttk.Combobox(cam_top, textvariable=cam_color, values=list(COLORS_HSV.keys()),
             state="readonly", width=8, font=(FONT, 9)).pack(side="left", padx=6)
# Explicit LIVE toggle (default OFF): the heavy 55 fps stream is pulled only
# when this is checked -> no accidental WiFi flood by clicking the tab.
cam_live = tk.BooleanVar(value=False)
tk.Checkbutton(cam_top, text="LIVE", variable=cam_live, bg=C_BG, fg=C_ERR,
               selectcolor="#fff", activebackground=C_BG,
               font=(FONT, 9, "bold")).pack(side="right")

cam_canvas = tk.Canvas(tab_cam, bg="#000", highlightthickness=1,
                       highlightbackground=C_LINE, width=CAM_W, height=CAM_H)
cam_canvas.pack(padx=12, pady=(4, 6))

cam_bar = tk.Frame(tab_cam, bg=C_BG)
cam_bar.pack(fill="x", padx=12)
cam_view = tk.IntVar(value=0)          # 0 Raw, 1 Detection, 2 Mask
CAM_MODES = ["Raw", "Color detection", "Mask"]
cam_mode_lbl = tk.Label(cam_bar, text=CAM_MODES[0], bg=C_BG, fg=C_ACCENT,
                        font=(FONT, 9, "bold"), width=16, anchor="w")
cam_mode_lbl.pack(side="right")
tk.Scale(cam_bar, from_=0, to=2, orient="horizontal", variable=cam_view,
         showvalue=False, resolution=1, length=300, bg=C_BG, fg=C_TXT,
         troughcolor=C_PANE2, highlightthickness=0, sliderrelief="flat",
         activebackground=C_ACCENT,
         command=lambda _v: cam_mode_lbl.config(
             text=CAM_MODES[int(cam_view.get())])).pack(side="left")
cam_info = tk.Label(tab_cam, text="", bg=C_BG, fg=C_SUB, font=(MONO, 9))
cam_info.pack(pady=(4, 0))

# --- Tab: Lidar ---
LID = 460
tab_lid = tk.Frame(nb, bg=C_BG)
nb.add(tab_lid, text="   Lidar   ")
lid_top = tk.Frame(tab_lid, bg=C_BG)
lid_top.pack(fill="x", padx=12, pady=(10, 4))
tk.Label(lid_top, text="Robot", bg=C_BG, fg=C_SUB, font=(FONT, 9)).pack(side="left")
lid_robot = tk.IntVar(value=1)
lid_robot.trace_add("write", lambda *_: map_pts.clear())
for i in ROBOTS:
    tk.Radiobutton(lid_top, text=f"t{i}", variable=lid_robot, value=i,
                   bg=C_BG, fg=C_TXT, selectcolor="#fff",
                   activebackground=C_BG, font=(FONT, 9)).pack(side="left")
lid_live = tk.BooleanVar(value=False)
tk.Checkbutton(lid_top, text="LIVE", variable=lid_live, bg=C_BG, fg=C_ERR,
               selectcolor="#fff", activebackground=C_BG,
               font=(FONT, 9, "bold")).pack(side="right")
tk.Button(lid_top, text="Clear map", command=lambda: map_pts.clear(),
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 8), cursor="hand2",
          activebackground=C_PANE2, padx=8).pack(side="right")

lid_canvas = tk.Canvas(tab_lid, bg="#fbfbfa", highlightthickness=1,
                       highlightbackground=C_LINE, width=LID, height=LID)
lid_canvas.pack(padx=12, pady=(4, 6))

lid_bar = tk.Frame(tab_lid, bg=C_BG)
lid_bar.pack(fill="x", padx=12)
lid_view = tk.IntVar(value=0)          # 0 Points, 1 Sectors, 2 Map
LID_MODES = ["Points", "Sectors", "Map"]
lid_mode_lbl = tk.Label(lid_bar, text=LID_MODES[0], bg=C_BG, fg=C_ACCENT,
                        font=(FONT, 9, "bold"), width=10, anchor="w")
lid_mode_lbl.pack(side="right")
tk.Scale(lid_bar, from_=0, to=2, orient="horizontal", variable=lid_view,
         showvalue=False, resolution=1, length=300, bg=C_BG, fg=C_TXT,
         troughcolor=C_PANE2, highlightthickness=0, sliderrelief="flat",
         activebackground=C_ACCENT,
         command=lambda _v: lid_mode_lbl.config(
             text=LID_MODES[int(lid_view.get())])).pack(side="left")
lid_info = tk.Label(tab_lid, text="", bg=C_BG, fg=C_SUB, font=(MONO, 9))
lid_info.pack(pady=(4, 0))
map_pts = deque(maxlen=4000)           # accumulated world points for Map view

# --- Tab: Topics ---
tab_top = tk.Frame(nb, bg=C_BG)
nb.add(tab_top, text="   Topics   ")
topics_txt = tk.Text(tab_top, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9),
                     state="disabled", wrap="none", padx=12, pady=10)
topics_txt.pack(fill="both", expand=True)
topics_txt.tag_configure("head", foreground=C_ACCENT, font=(MONO, 9, "bold"))
topics_txt.tag_configure("dbl", foreground=C_ERR)
topics_txt.tag_configure("info", foreground=C_TXT)

# --- Tab: Formation (cascade chain editor) ---
tab_form = tk.Frame(nb, bg=C_BG)
nb.add(tab_form, text="   Formation   ")
tk.Label(tab_form, text="CASCADE CHAIN", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", padx=16, pady=(16, 2))
tk.Label(tab_form, text="Each robot follows the helmet color of the robot "
         "ahead of it. Set who wears which color:",
         bg=C_BG, fg=C_FAINT, font=(FONT, 8), wraplength=520,
         justify="left").pack(anchor="w", padx=16)

helmet_var, form_sw = {}, {}


def on_helmet(i):
    HELMETS[i] = helmet_var[i].get()
    hexc = HELMET_HEX.get(HELMETS[i], C_TXT)
    form_sw[i].config(fg=hexc)
    if i in helmet_sw:
        helmet_sw[i].config(fg=hexc)
    update_formation()
    refresh_info()


hrows = tk.Frame(tab_form, bg=C_BG)
hrows.pack(fill="x", padx=16, pady=(8, 4))
for i in ROBOTS:
    r = tk.Frame(hrows, bg=C_BG)
    r.pack(fill="x", pady=2)
    form_sw[i] = tk.Label(r, text="■", fg=HELMET_HEX[HELMETS[i]], bg=C_BG,
                          font=(FONT, 12))
    form_sw[i].pack(side="left", padx=(0, 6))
    tk.Label(r, text=f"tortuga{i} wears", width=15, anchor="w", bg=C_BG,
             fg=C_TXT, font=(FONT, 10)).pack(side="left")
    helmet_var[i] = tk.StringVar(value=HELMETS[i])
    cb = ttk.Combobox(r, textvariable=helmet_var[i], values=COLOR_NAMES,
                      state="readonly", width=8, font=(FONT, 9))
    cb.pack(side="left", padx=6)
    cb.bind("<<ComboboxSelected>>", lambda e, i=i: on_helmet(i))

tk.Frame(tab_form, bg=C_LINE, height=1).pack(fill="x", padx=16, pady=(10, 6))
tk.Label(tab_form, text="RESULTING CHAIN  (uses the selected robots, in order)",
         bg=C_BG, fg=C_FAINT, font=(FONT, 8, "bold")).pack(anchor="w", padx=16)
form_txt = tk.Text(tab_form, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9), height=6,
                   state="disabled", padx=12, pady=6, wrap="word")
form_txt.pack(fill="x", padx=12, pady=(2, 0))

frow = tk.Frame(tab_form, bg=C_BG)
frow.pack(fill="x", padx=16, pady=(12, 4))
tk.Button(frow, text="HSV Tuner…", command=open_hsv_tuner, bg=C_ACCENT,
          fg="white", bd=0, font=(FONT, 10, "bold"), cursor="hand2",
          activebackground=C_ACCENT_D, padx=14, pady=6).pack(side="left")
tk.Button(frow, text="Save chain", command=save_settings, bg=C_PANE, fg=C_TXT,
          bd=0, font=(FONT, 9), cursor="hand2", activebackground=C_PANE2,
          padx=12, pady=6).pack(side="left", padx=8)
tk.Label(tab_form, text="HSV Tuner: dial in each color filter live (camera + "
         "mask preview) and save the presets.",
         bg=C_BG, fg=C_FAINT, font=(FONT, 8), wraplength=520,
         justify="left").pack(anchor="w", padx=16, pady=(4, 0))


def update_formation():
    order = present_indices() or list(ROBOTS)
    form_txt.configure(state="normal")
    form_txt.delete("1.0", "end")
    for pos, i in enumerate(order):
        c = HELMETS[i]
        if pos == 0:
            form_txt.insert("end", f"tortuga{i}  ({c})   →  LEADER (drive ZQSD)\n")
        else:
            pj = order[pos - 1]
            form_txt.insert("end",
                            f"tortuga{i}  ({c})   →  follows tortuga{pj} "
                            f"({HELMETS[pj]})\n")
    form_txt.configure(state="disabled")


# --- Tab: Dataset (settings) ---
tab_ds = tk.Frame(nb, bg=C_BG)
nb.add(tab_ds, text="   Dataset   ")
tk.Label(tab_ds, text="RECORDING SETTINGS", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", padx=16, pady=(16, 2))
tk.Label(tab_ds, text="Applied when you start the Dataset mode.",
         bg=C_BG, fg=C_FAINT, font=(FONT, 8)).pack(anchor="w", padx=16)

dur_var = tk.StringVar(value="120")
seg_var = tk.StringVar(value="5")
batt_var = tk.StringVar(value="20")     # percent
disk_var = tk.StringVar(value="1.0")
load_settings()                          # override with saved config if present


def ds_field(label, var, unit, hint=""):
    row = tk.Frame(tab_ds, bg=C_BG)
    row.pack(fill="x", padx=16, pady=4)
    tk.Label(row, text=label, bg=C_BG, fg=C_TXT, font=(FONT, 10),
             width=22, anchor="w").pack(side="left")
    tk.Entry(row, textvariable=var, width=8, font=(MONO, 10), bg="#fff",
             fg=C_TXT, relief="solid", bd=1, justify="right").pack(side="left")
    tk.Label(row, text=" " + unit, bg=C_BG, fg=C_SUB,
             font=(FONT, 9), width=4, anchor="w").pack(side="left")
    if hint:
        tk.Label(row, text=hint, bg=C_BG, fg=C_FAINT,
                 font=(FONT, 8)).pack(side="left")


tk.Frame(tab_ds, bg=C_LINE, height=1).pack(fill="x", padx=16, pady=(10, 6))
ds_field("Recording duration", dur_var, "min", "0 = unlimited")
ds_field("Segment length", seg_var, "min", "video file size")
tk.Frame(tab_ds, bg=C_LINE, height=1).pack(fill="x", padx=16, pady=(10, 6))
tk.Label(tab_ds, text="AUTO-STOP GUARDS", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", padx=16, pady=(2, 4))
ds_field("Stop if battery <", batt_var, "%", "0 = off")
ds_field("Stop if storage <", disk_var, "GB", "0 = off")

shutdown_var = tk.BooleanVar(value=False)
srow = tk.Frame(tab_ds, bg=C_BG)
srow.pack(fill="x", padx=16, pady=(8, 2))
tk.Checkbutton(srow, text="Power OFF robots on auto-stop (poweroff)",
               variable=shutdown_var, bg=C_BG, fg=C_TXT, selectcolor="#fff",
               activebackground=C_BG, font=(FONT, 10)).pack(side="left")

tk.Button(tab_ds, text="Save settings", command=save_settings,
          bg=C_ACCENT, fg="white", bd=0, font=(FONT, 10, "bold"),
          cursor="hand2", activebackground=C_ACCENT_D, pady=7).pack(
    anchor="w", padx=16, pady=(16, 4))
tk.Label(tab_ds, text="When a guard trips: recording is always closed cleanly "
         "(segments valid). The robots are only POWERED OFF if the box above is "
         "checked — otherwise the bringup stays up so you can pull right away.",
         bg=C_BG, fg=C_FAINT, font=(FONT, 8), wraplength=520,
         justify="left").pack(anchor="w", padx=16, pady=(10, 0))

# status bar
status = tk.StringVar(value="Ready.")
tk.Label(root, textvariable=status, bg=C_PANE, fg=C_SUB, anchor="w",
         font=(FONT, 9), padx=16, pady=5).pack(fill="x", side="bottom")


# ---------------- Dynamic renders ----------------
def update_state_lights():
    if hub is None:
        return
    now = time.time()

    def fresh(d, i):                 # data received in the last 3 s = alive
        return (i in d) and (now - d[i] < 3.0)

    for i in ROBOTS:
        checks = (("M", fresh(hub.t_odom, i)),   # bringup/motors publishing odom
                  ("L", fresh(hub.t_scan, i)),   # lidar publishing scan
                  ("C", fresh(hub.t_cam, i)))    # camera_info (camera alive)
        for key, on in checks:
            state_dot[i][key].config(fg="white" if on else C_FAINT,
                                     bg=C_OK if on else C_PANE2)


def render_cam():
    if hub is None:
        return
    if not cam_live.get():
        cam_canvas.delete("all")
        cam_canvas.create_text(CAM_W // 2, CAM_H // 2,
                               text="Camera stream OFF — check LIVE to view",
                               fill="#888", font=(FONT, 11))
        cam_info.config(text="(no WiFi bandwidth used while OFF)")
        return
    i = cam_robot.get()
    bgr = hub.frames.get(i)
    if bgr is None:
        cam_canvas.delete("all")
        cam_canvas.create_text(CAM_W // 2, CAM_H // 2, text="Waiting for camera…",
                               fill="#888", font=(FONT, 11))
        cam_info.config(text="")
        return
    h, w = bgr.shape[:2]
    view = int(cam_view.get())
    color = cam_color.get()
    out = bgr
    tag = "raw"
    if view == 1:                       # color detection overlay
        out = bgr.copy()
        mask = color_mask(bgr, color)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        col = DRAW_BGR.get(color, (0, 220, 0))
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 200:
                x, y, ww, hh = cv2.boundingRect(c)
                cv2.rectangle(out, (x, y), (x + ww, y + hh), col, 2)
                cx, cy = x + ww // 2, y + hh // 2
                cv2.drawMarker(out, (cx, cy), col, cv2.MARKER_CROSS, 18, 2)
        tag = f"detection ({color})"
    elif view == 2:                     # binary mask
        mask = color_mask(bgr, color)
        out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        tag = f"mask ({color})"
    scale = min(CAM_W / w, CAM_H / h)
    disp = cv2.resize(out, (int(w * scale), int(h * scale)))
    img = to_photo(disp)
    if img is not None:
        cam_canvas._img = img
        cam_canvas.delete("all")
        cam_canvas.create_image(CAM_W // 2, CAM_H // 2, image=img)
    cam_info.config(text=f"tortuga{i}   {w}×{h}   {tag}")


def _lidar_base_img():
    img = np.full((LID, LID, 3), 251, np.uint8)
    return img


def render_lidar():
    if hub is None:
        return
    if not lid_live.get():
        lid_canvas.delete("all")
        lid_canvas.create_text(LID // 2, LID // 2,
                               text="Lidar view OFF — check LIVE to view",
                               fill="#999", font=(FONT, 11))
        lid_info.config(text="")
        return
    i = lid_robot.get()
    msg = hub.scans.get(i)
    view = int(lid_view.get())
    if msg is None or not msg.ranges or msg.angle_increment == 0.0:
        lid_canvas.delete("all")
        lid_canvas.create_text(LID // 2, LID // 2, text="Waiting for scan…",
                               fill="#999", font=(FONT, 11))
        lid_info.config(text="")
        return
    r = np.array(msg.ranges, dtype=float)
    r[np.isinf(r) | np.isnan(r)] = 0.0
    ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    cx = cy = LID // 2

    if view == 2:                       # ---- Map (accumulated in world frame) ----
        img = _lidar_base_img()
        pose = hub.odom.get(i)
        good = (r > 0.06) & (r < 3.5)
        if pose is not None and np.any(good):
            x, y, th = pose
            px = r[good] * np.cos(ang[good])
            py = r[good] * np.sin(ang[good])
            wx = x + px * math.cos(th) - py * math.sin(th)
            wy = y + px * math.sin(th) + py * math.cos(th)
            for k in range(0, len(wx), 2):     # downsample when adding
                map_pts.append((wx[k], wy[k]))
        if map_pts:
            arr = np.array(map_pts)
            mnx, mxx = arr[:, 0].min(), arr[:, 0].max()
            mny, mxy = arr[:, 1].min(), arr[:, 1].max()
            span = max(mxx - mnx, mxy - mny, 1.0)
            sc = (LID - 60) / span
            ox = (mnx + mxx) / 2.0
            oy = (mny + mxy) / 2.0
            for wx0, wy0 in map_pts:
                u = int(cx + (wx0 - ox) * sc)
                vv = int(cy - (wy0 - oy) * sc)
                if 0 <= u < LID and 0 <= vv < LID:
                    cv2.circle(img, (u, vv), 1, (170, 120, 40), -1)
            if hub.odom.get(i) is not None:
                x, y, th = hub.odom[i]
                u = int(cx + (x - ox) * sc)
                vv = int(cy - (y - oy) * sc)
                cv2.circle(img, (u, vv), 5, (60, 60, 220), -1)
                cv2.line(img, (u, vv),
                         (int(u + 16 * math.cos(-th)), int(vv + 16 * math.sin(-th))),
                         (60, 60, 220), 2)
        lid_info.config(text=f"tortuga{i}   map · {len(map_pts)} pts   (Clear to reset)")
        img_ppm = to_photo(img)
        if img_ppm is not None:
            lid_canvas._img = img_ppm
            lid_canvas.delete("all")
            lid_canvas.create_image(cx, cy, image=img_ppm)
        return

    img = _lidar_base_img()
    sc = 82.0
    for rad in (0.5, 1.0, 1.5, 2.0):
        cv2.circle(img, (cx, cy), int(rad * sc), (227, 227, 224), 1)
    cv2.circle(img, (cx, cy), int(SAFETY * sc), (62, 62, 224), 1)

    if view == 1:                       # ---- Sectors ----
        for lbl, ctr in (("F", 0), ("L", 90), ("R", -90), ("B", 180)):
            d = sector_min(msg, ctr, 35)
            a = math.radians(ctr)
            ux, uy = -math.sin(a), -math.cos(a)
            col = (62, 62, 224) if d < SAFETY + 0.05 else (179, 111, 47)
            ex, ey = int(cx + ux * min(d, 2.0) * sc), int(cy + uy * min(d, 2.0) * sc)
            cv2.line(img, (cx, cy), (ex, ey), col, 3)
            tx, ty = int(cx + ux * 2.15 * sc), int(cy + uy * 2.15 * sc)
            txt = "-" if d >= 90 else f"{d:.2f}"
            cv2.putText(img, f"{lbl}:{txt}",
                        (tx - 26, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1,
                        cv2.LINE_AA)
        front = sector_min(msg, 0, 20)
        lid_info.config(text=f"tortuga{i}   sectors   front "
                             + ("—" if front >= 90 else f"{front:.2f} m"))
    else:                               # ---- Points ----
        good = (r > 0.06) & (r < 2.0)
        for a, d in zip(ang[good], r[good]):
            u = int(cx - math.sin(a) * d * sc)
            vv = int(cy - math.cos(a) * d * sc)
            col = (62, 62, 224) if d < SAFETY + 0.05 else (179, 111, 47)
            cv2.circle(img, (u, vv), 1, col, -1)
        front = sector_min(msg, 0, 20)
        lid_info.config(text=f"tortuga{i}   points   front "
                             + ("—" if front >= 90 else f"{front:.2f} m"))

    cv2.circle(img, (cx, cy), 3, (47, 42, 43), -1)
    cv2.line(img, (cx, cy), (cx, cy - 24), (179, 111, 47), 2)
    img_ppm = to_photo(img)
    if img_ppm is not None:
        lid_canvas._img = img_ppm
        lid_canvas.delete("all")
        lid_canvas.create_image(cx, cy, image=img_ppm)


def render_topics():
    if hub is None:
        return
    topics_txt.configure(state="normal")
    topics_txt.delete("1.0", "end")
    for i in ROBOTS:
        tp = sorted(hub.topics_by_robot.get(i, set()))
        if not tp:
            continue
        topics_txt.insert("end", f"tortuga{i}\n", "head")
        for name in tp:
            dbl = f"/tortuga{i}/tortuga{i}/" in name
            short = name.replace(f"/tortuga{i}", "", 1) or "/"
            tag = "dbl" if dbl else "info"
            flag = "  ⚠ DOUBLE NS" if dbl else ""
            topics_txt.insert("end", f"   {short}{flag}\n", tag)
        topics_txt.insert("end", "\n")
    if not any(hub.topics_by_robot.values()):
        topics_txt.insert("end", "No topic detected.\n"
                          "Start a robot, then wait ~3 s.\n", "info")
    topics_txt.configure(state="disabled")


def tick():
    try:
        update_state_lights()
        cur = nb.index(nb.select())     # 0 Log, 1 Cam, 2 Lidar, 3 Topics, 4 Dataset
        # Heavy camera stream: only when the Camera tab is open AND its LIVE
        # box is checked, for the displayed robot -> no accidental WiFi flood.
        if hub is not None:
            hub.want_cam = cam_robot.get() if (cur == 1 and cam_live.get()) \
                else None
        if cur == 1:
            render_cam()
        elif cur == 2:
            render_lidar()
        elif cur == 3:
            render_topics()
    except Exception:
        pass
    root.after(200, tick)


def deferred_start():
    if hub is not None:
        hub.start()
    refresh_state()
    log("Interface ready.", "ok")


hub = Hub() if ROS_OK else None
refresh_info()
if not ROS_OK:
    log("ROS not sourced: camera/lidar/topics inspection inactive.", "warn")
root.after(300, deferred_start)
root.after(400, tick)
root.mainloop()
