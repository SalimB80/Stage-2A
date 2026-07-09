#!/usr/bin/env python3
"""
Launcher TurtleBot3 v6 — tout a la demande, une fenetre.

Par robot (une ligne) : coche presence | pastille ping | espace disque live |
casque | boutons  CAM  LIDAR  MAP  ZQSD.
  CAM   : ouvre/ferme le retour camera (carres de detection) de CE robot.
  LIDAR : ouvre/ferme le radar lidar de CE robot (seuil securite 0.16).
  MAP   : lance/arrete la cartographie SLAM de CE robot, puis sauvegarde.
  ZQSD  : teleop clavier de CE robot en un clic (meme non coche).

Modes : Cascade | Errance | Dataset (video+lidar).
Donnees : PULL (recup+libere) | Copier | Supprimer | Espace.
Monitoring : journal horodate des actions.

Retour cam/lidar : flux compresses, resolution auto des topics (ns simple/double).
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import threading
import time
import math

ROS_OK = True
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, CompressedImage
    import numpy as np
    import cv2
except Exception:
    ROS_OK = False

ROBOTS = {
    1: ("tortuga1", "192.168.0.201"),
    2: ("tortuga2", "192.168.0.202"),
    3: ("tortuga3", "192.168.0.203"),
    4: ("tortuga4", "192.168.0.204"),
}
HELMETS = {1: "jaune", 2: "rouge", 3: "cyan", 4: "jaune"}
FORMATION_BEARINGS = {
    "colonne": [0, 0, 0], "ligne": [30, -30, 30],
    "triangle": [25, -25, 0], "carre": [20, -20, 0],
}
TARGET_DISTANCE = 0.6
SAFETY = 0.16
PW = "1234"
ROS_DOMAIN_ID = 30

DEBUG_COLORS = {
    "jaune": {"ranges": [([20, 80, 80], [35, 255, 255])], "bgr": (0, 220, 255)},
    "rouge": {"ranges": [([0, 100, 80], [8, 255, 255]),
                         ([172, 100, 80], [179, 255, 255])], "bgr": (60, 60, 230)},
    "cyan":  {"ranges": [([85, 80, 80], [100, 255, 255])], "bgr": (230, 200, 60)},
}
KILL_CMD = ("pkill -9 -f '[r]os2 launch'; pkill -9 -f '[r]obot_full.launch'; "
            "pkill -9 -f '[r]obot_dataset.launch'; "
            "pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'; sleep 1; "
            "pkill -9 -f -- '[-]-ros-args'; sleep 0.5; "
            "pkill -9 -f -- '[-]-ros-args'; true")
CHECK_CMD = "pgrep -fc -- '[-]-ros-args' || true"
DF_CMD = "df -h ~ | tail -1 | awk '{print $4}'"

C_BG = "#15151d"; C_CARD = "#212130"; C_CARD2 = "#2a2a3c"; C_TXT = "#e8e8f2"
C_SUB = "#9797b0"; C_ACCENT = "#00CED1"; C_DANGER = "#e05555"; C_OK = "#37c871"
C_BAD = "#e05555"; C_UNK = "#5a5a70"; C_WARN = "#e8b44d"; C_GREEN = "#2f9d5a"
HELMET_HEX = {"jaune": "#e8d44d", "rouge": "#e05555", "cyan": "#4dd0e1"}
DATASET_SH = "./src/formation_control/tools/dataset_tools.sh"


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
    for att in (["wt.exe", "wsl.exe", "-e", "bash", "-lc", bash_cmd],
                ["cmd.exe", "/c", "start", "", "wsl.exe", "-e", "bash", "-lc",
                 bash_cmd]):
        try:
            subprocess.Popen(att, cwd="/mnt/c/",
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except FileNotFoundError:
            continue
    log(fallback)
    return False


class Hub:
    def __init__(self):
        self.frames = {}
        self.scans = {}
        self.subbed = set()
        self.node = None
        self.started = False
        self.topics_by_robot = {i: set() for i in ROBOTS}

    def start(self):
        # Demarre le hub ROS APRES l'affichage de la fenetre (non bloquant).
        if self.started or not ROS_OK:
            return
        self.started = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = RosNode("gui_hub")
            self.node.create_timer(3.0, self._discover)
            rclpy.spin(self.node)
        except Exception as e:
            print(f"[hub ROS desactive] {e}")

    def _discover(self):
        if self.node is None:
            return
        # Recense les topics presents par robot pour les voyants d'etat.
        # (la presence d'un topic = le noeud correspondant tourne)
        seen = {i: set() for i in ROBOTS}
        for name, types in self.node.get_topic_names_and_types():
            for idx, (rn, _) in ROBOTS.items():
                if f"/{rn}/" not in name:
                    continue
                seen[idx].add(name)
                if name in self.subbed:
                    continue
                if name.endswith("image_raw/compressed") and \
                        "sensor_msgs/msg/CompressedImage" in types:
                    self.node.create_subscription(
                        CompressedImage, name,
                        lambda m, i=idx: self._img(i, m), qos_profile_sensor_data)
                    self.subbed.add(name)
                elif name.endswith("/scan") and \
                        "sensor_msgs/msg/LaserScan" in types:
                    self.node.create_subscription(
                        LaserScan, name,
                        lambda m, i=idx: self.scans.__setitem__(i, m),
                        qos_profile_sensor_data)
                    self.subbed.add(name)
        self.topics_by_robot = seen

    def _img(self, idx, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            f = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if f is not None:
                self.frames[idx] = f
        except Exception:
            pass


def detect_boxes(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    for name, spec in DEBUG_COLORS.items():
        mask = None
        for lo, hi in spec["ranges"]:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 600:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), spec["bgr"], 2)
                cv2.putText(frame, f"{name} {int(cv2.contourArea(c))}",
                            (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, spec["bgr"], 1)
    return frame


def sector_min(msg, center_deg, half_deg):
    # Garde-fou : scan vide ou malforme (increment nul) -> pas de mesure.
    if not msg.ranges or msg.angle_increment == 0.0:
        return 99.0
    r = np.array(msg.ranges)
    r[np.isinf(r) | np.isnan(r)] = 99.0
    n = len(r); two_pi = 2 * math.pi
    if n == 0:
        return 99.0
    a = math.radians(center_deg)
    while a < msg.angle_min:
        a += two_pi
    while a >= msg.angle_min + two_pi:
        a -= two_pi
    c = int((a - msg.angle_min) / msg.angle_increment) % n
    h = max(1, int(math.radians(half_deg) / msg.angle_increment))
    idxs = np.arange(c - h, c + h + 1) % n
    w = r[idxs]; valid = w[(w > 0.06) & (w < 90.0)]
    return float(np.min(valid)) if len(valid) else 99.0


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


def log(msg):
    ts = time.strftime("%H:%M:%S")
    journal.configure(state="normal")
    journal.insert("end", f"[{ts}] {msg}\n")
    journal.see("end")
    journal.configure(state="disabled")
    status.set(msg)


def refresh_info(*_):
    m = mode_var.get()
    launch_btn.config(text={"cascade": "LANCER CASCADE",
                            "errance": "LANCER ERRANCE",
                            "dataset": "LANCER DATASET"}[m])
    form_combo.configure(state="readonly" if m == "cascade" else "disabled")
    present = present_indices()
    if not present:
        info.set("Coche les robots a utiliser")
        return
    if m == "cascade":
        ch = build_chain()
        parts = [f"t{ch[0][0]}=LEADER"] + \
                [f"t{r}->{c}({b:+.0f})" for r, _, c, b in ch[1:]]
        info.set("  ".join(parts))
    elif m == "errance":
        info.set("Errance autonome (securite 0.16 m) : "
                 + ", ".join(f"t{i}" for i in present))
    else:
        info.set("DATASET video+lidar : " + ", ".join(f"t{i}" for i in present))


def ping_space_async():
    log("Rafraichissement etat + espace disque...")
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
            free = "--"
            if ok:
                try:
                    r = subprocess.run(ssh_args(i, DF_CMD, timeout=4),
                                       capture_output=True, text=True, timeout=8)
                    free = (r.stdout or "--").strip() or "--"
                except Exception:
                    free = "?"
            res[i] = (ok, free)
        root.after(0, lambda: apply_state(res))
    threading.Thread(target=work, daemon=True).start()


def apply_state(res):
    for i, (ok, free) in res.items():
        dots[i].itemconfig("dot", fill=C_OK if ok else C_BAD)
        space_lbl[i].config(text=free)
    log("Etat a jour.")


def bringup_start():
    """COUCHE 1 : demarre le materiel (moteurs+lidar+camera) des robots coches.
    Reste allume ; on lance ensuite un comportement par-dessus."""
    present = present_indices()
    if not present:
        log("Coche au moins un robot.")
        return
    for i in present:
        ssh_bg(i, robot_env() +
               f"ros2 launch formation_control robot_bringup.launch.py "
               f"namespace:=tortuga{i}")
    log("BRINGUP lance : " + ", ".join(f"t{i}" for i in present)
        + " (attends ~15 s que capteurs demarrent)")


def bringup_stop():
    """Coupe SEULEMENT le bringup (couche 1) des robots coches."""
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[r]obot_bringup.launch'; "
                  "pkill -9 -f '[t]urtlebot3_ros'; pkill -9 -f '[s]ingle_coin'; "
                  "pkill -9 -f '[c]amera_node'; pkill -9 -f '[r]obot_state'")
    log("BRINGUP coupe : " + ", ".join(f"t{i}" for i in present))


def behavior_start():
    """COUCHE 2 : lance le comportement choisi SUR le bringup deja actif.
    Ne touche pas au bringup -> changement de mode a chaud."""
    present = present_indices()
    if not present:
        log("Coche au moins un robot.")
        return
    m = mode_var.get()
    # on coupe d'abord tout ancien comportement (mais PAS le bringup)
    behavior_stop(silent=True)
    if m == "cascade":
        for rob, role, color, bear in build_chain():
            if role == "leader":
                log(f"t{rob} = leader (a piloter en ZQSD, pas de noeud).")
                continue
            ssh_bg(rob, robot_env() +
                   f"ros2 launch formation_control robot_behavior.launch.py "
                   f"namespace:=tortuga{rob} mode:=cascade role:=tracker "
                   f"robot_index:={rob} target_color:={color} "
                   f"desired_bearing:={bear} target_distance:={TARGET_DISTANCE}")
        log(f"CASCADE lancee ({form_var.get()}).")
    else:
        rec = "true" if m == "dataset" else "false"
        for i in present:
            ssh_bg(i, robot_env() +
                   f"ros2 launch formation_control robot_behavior.launch.py "
                   f"namespace:=tortuga{i} mode:={m} record:={rec}")
        log(("DATASET lance (enregistre !)" if m == "dataset"
             else "ERRANCE lancee") + " : " + ", ".join(f"t{i}" for i in present))


def behavior_stop(silent=False):
    """Coupe SEULEMENT le comportement (wander/tracker/recorder/bag),
    en gardant le bringup vivant."""
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[r]obot_behavior.launch'; "
                  "pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
                  "pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'")
    if not silent:
        log("COMPORTEMENT coupe (bringup conserve) : "
            + ", ".join(f"t{i}" for i in present))


def apply_formation():
    if mode_var.get() != "cascade":
        log("Formation : cascade uniquement.")
        return
    for rob, _, _, bear in build_chain()[1:]:
        subprocess.Popen(["bash", "-c", pc_env() +
            f"ros2 param set /tortuga{rob}/tracker desired_bearing {bear}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Bearings appliques ({form_var.get()}).")


def teleop_robot(idx):
    if open_terminal(pc_env() +
                     f"ros2 run formation_control teleop_zqsd tortuga{idx}",
                     f"Manuel : teleop_zqsd tortuga{idx}"):
        log(f"Teleop ZQSD tortuga{idx}.")


map_running = set()


def toggle_map(idx):
    if idx in map_running:
        open_terminal(pc_env() +
            f"echo 'Sauvegarde carte tortuga{idx}...'; "
            f"mkdir -p ~/formation_ws/maps; "
            f"ros2 run nav2_map_server map_saver_cli -t /tortuga{idx}/map "
            f"-f ~/formation_ws/maps/map_tortuga{idx}_$(date +%Y%m%d_%H%M); "
            f"echo 'Carte sauvee dans ~/formation_ws/maps/. Entree.'; read",
            "Manuel : map_saver_cli")
        ssh_bg(idx, "pkill -TERM -f '[s]lam_toolbox'")
        map_running.discard(idx)
        map_btn[idx].config(text="MAP", bg="#2e2e44")
        log(f"Carte tortuga{idx} sauvegardee, SLAM arrete.")
    else:
        ssh_bg(idx, robot_env() +
               f"ros2 launch formation_control robot_dataset.launch.py "
               f"namespace:=tortuga{idx} record:=false slam:=true")
        map_running.add(idx)
        map_btn[idx].config(text="MAP*", bg=C_GREEN)
        log(f"SLAM tortuga{idx} demarre — promene le robot puis reclique MAP.")


def ds_pull():
    idx = " ".join(map(str, targets()))
    open_terminal(pc_env() + f"cd ~/formation_ws && {DATASET_SH} pull {idx}; "
                  "echo; echo '=== Termine. Entree ==='; read",
                  "Manuel : dataset_tools.sh pull")
    log(f"PULL lance (robots {idx}).")


def ds_collect():
    idx = " ".join(map(str, targets()))
    open_terminal(pc_env() + f"cd ~/formation_ws && {DATASET_SH} collect {idx}; "
                  "echo; echo '=== Copie faite. Entree ==='; read",
                  "Manuel : dataset_tools.sh collect")
    log(f"Copie lancee (robots {idx}).")


def ds_clean():
    idx = " ".join(map(str, targets()))
    if not messagebox.askyesno("Supprimer",
            f"Supprimer TOUTES les donnees (video+lidar) sur {idx} ?\n\n"
            "Verifie d'abord le PULL/COLLECT !"):
        return
    open_terminal(pc_env() + f"cd ~/formation_ws && echo oui | "
                  f"{DATASET_SH} clean {idx}; echo '=== Fait. Entree ==='; read",
                  "Manuel : dataset_tools.sh clean")
    log(f"Suppression lancee (robots {idx}).")


def build_deploy():
    log("Build + deploy en cours... (peut prendre 1-2 min)")
    idxs = " ".join(map(str, present_indices())) or "1 2 3 4"
    def work():
        try:
            r = subprocess.run(
                ["bash", "-c", 
                 "cd ~/formation_ws && colcon build --symlink-install && "
                 "source install/setup.bash && "
                 f"./src/formation_control/tools/deploy_build.sh {idxs}"],
                capture_output=True, text=True, timeout=180)
            if r.returncode == 0:
                root.after(0, lambda: log("Build + deploy OK ✓"))
            else:
                out = (r.stdout + r.stderr)[-300:]
                root.after(0, lambda: log(f"Build ERROR : {out}"))
        except subprocess.TimeoutExpired:
            root.after(0, lambda: log("Build timeout (>3 min)"))
        except Exception as e:
            root.after(0, lambda: log(f"Build exception : {e}"))
    threading.Thread(target=work, daemon=True).start()


def stop_everything():
    log("STOP : kill sur tous les robots...")
    stop_btn.config(state="disabled")
    map_running.clear()
    for i in ROBOTS:
        map_btn[i].config(text="MAP", bg="#2e2e44")
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
                rep.append(f"t{i}:" + ("ok" if n == "0" else f"{n}!"))
            except Exception:
                rep.append(f"t{i}:off")
        root.after(0, lambda: (stop_btn.config(state="normal"),
                               log("ARRET — " + " ".join(rep))))
    threading.Thread(target=work, daemon=True).start()


root = tk.Tk()
root.title("TurtleBot3 Control v6")
root.geometry("560x900")
root.configure(bg=C_BG)
style = ttk.Style(root); style.theme_use("clam")
style.configure("TCombobox", fieldbackground=C_CARD2, background=C_CARD2,
                foreground=C_TXT)


def card(parent, title):
    f = tk.Frame(parent, bg=C_CARD)
    f.pack(fill="x", padx=10, pady=(7, 0))
    tk.Label(f, text=title, bg=C_CARD, fg=C_SUB,
             font=("", 9, "bold")).pack(anchor="w", padx=10, pady=(6, 2))
    inner = tk.Frame(f, bg=C_CARD)
    inner.pack(fill="x", padx=10, pady=(0, 8))
    return inner


def sbtn(parent, txt, cmd, bg="#2e2e44", w=None):
    return tk.Button(parent, text=txt, command=cmd, bg=bg, fg=C_TXT, bd=0,
                     cursor="hand2", activebackground="#3a3a52",
                     activeforeground=C_TXT, padx=6, pady=1, font=("", 8),
                     width=w)


tk.Label(root, text="TurtleBot3 Control", bg=C_BG, fg=C_TXT,
         font=("", 15, "bold")).pack(pady=(9, 0))

rf = card(root, "ROBOTS   —   CAM / LIDAR / MAP / ZQSD a la demande")
present_vars, dots, space_lbl, map_btn = {}, {}, {}, {}
state_lbl, mode_lbl = {}, {}
cam_show, lidar_show = {}, {}
for i in ROBOTS:
    row = tk.Frame(rf, bg=C_CARD)
    row.pack(fill="x", pady=2)
    c = tk.Canvas(row, width=12, height=12, bg=C_CARD, highlightthickness=0)
    c.create_oval(2, 2, 10, 10, fill=C_UNK, outline="", tags="dot")
    c.pack(side="left", padx=(0, 3))
    dots[i] = c
    hc = tk.Canvas(row, width=12, height=12, bg=C_CARD, highlightthickness=0)
    hc.create_rectangle(2, 2, 10, 10, fill=HELMET_HEX[HELMETS[i]], outline="")
    hc.pack(side="left", padx=(0, 4))
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_info)
    tk.Checkbutton(row, text=f"t{i}", variable=v, bg=C_CARD, fg=C_TXT,
                   selectcolor=C_BG, activebackground=C_CARD,
                   activeforeground=C_TXT, width=3).pack(side="left")
    space_lbl[i] = tk.Label(row, text="--", bg=C_CARD, fg=C_WARN,
                            font=("Courier", 8), width=5)
    space_lbl[i].pack(side="left", padx=(0, 4))
    # Voyants d'etat : M(oteurs) L(idar) C(amera) + mode
    st = tk.Frame(row, bg=C_CARD)
    st.pack(side="left", padx=(0, 4))
    state_lbl[i] = {}
    for key in ("M", "L", "C"):
        lb = tk.Label(st, text=key, bg="#333", fg="#666",
                      font=("Courier", 8, "bold"), width=2)
        lb.pack(side="left", padx=1)
        state_lbl[i][key] = lb
    mode_lbl[i] = tk.Label(st, text="--", bg=C_CARD, fg=C_SUB,
                           font=("Courier", 7), width=7)
    mode_lbl[i].pack(side="left", padx=(3, 0))
    cam_show[i] = tk.BooleanVar(value=False)
    lidar_show[i] = tk.BooleanVar(value=False)
    sbtn(row, "CAM", lambda i=i: (cam_show[i].set(not cam_show[i].get()),
         open_view(i)), bg="#2e2e44").pack(side="left", padx=1)
    sbtn(row, "LIDAR", lambda i=i: (lidar_show[i].set(not lidar_show[i].get()),
         open_view(i)), bg="#2e2e44").pack(side="left", padx=1)
    map_btn[i] = sbtn(row, "MAP", lambda i=i: toggle_map(i), bg="#2e2e44")
    map_btn[i].pack(side="left", padx=1)
    sbtn(row, "ZQSD", lambda i=i: teleop_robot(i), bg="#3a2e44").pack(
        side="left", padx=1)

sbtn(rf, "Rafraichir etat + espace disque", ping_space_async,
     bg=C_CARD2).pack(fill="x", pady=(4, 0))

mf = card(root, "MODE DE LANCEMENT")
mode_var = tk.StringVar(value="cascade")
mode_var.trace_add("write", refresh_info)
for val, txt in (("cascade", "Cascade — leader pilote + suiveurs couleur"),
                 ("errance", "Errance — aleatoire autonome (securite 0.16 m)"),
                 ("dataset", "Dataset — errance + enregistrement video+lidar")):
    tk.Radiobutton(mf, text=txt, variable=mode_var, value=val, bg=C_CARD,
                   fg=C_TXT, selectcolor=C_BG, activebackground=C_CARD,
                   activeforeground=C_TXT, anchor="w").pack(fill="x")
frow = tk.Frame(mf, bg=C_CARD)
frow.pack(fill="x", pady=(3, 0))
tk.Label(frow, text="Formation", bg=C_CARD, fg=C_SUB).pack(side="left")
form_var = tk.StringVar(value="colonne")
form_var.trace_add("write", refresh_info)
form_combo = ttk.Combobox(frow, textvariable=form_var,
                          values=list(FORMATION_BEARINGS.keys()),
                          state="readonly", width=10)
form_combo.pack(side="left", padx=6)
sbtn(frow, "Appliquer a chaud", apply_formation).pack(side="left")

info = tk.StringVar(value="")
tk.Label(root, textvariable=info, bg=C_BG, fg=C_ACCENT, wraplength=530,
         font=("", 9)).pack(pady=(5, 0))

af = tk.Frame(root, bg=C_BG)
af.pack(fill="x", padx=10, pady=(6, 0))

# Couche 1 : bringup
b1 = tk.Frame(af, bg=C_BG); b1.pack(fill="x")
tk.Label(b1, text="1. MATERIEL", bg=C_BG, fg=C_SUB,
         font=("", 8, "bold")).pack(anchor="w")
b1r = tk.Frame(af, bg=C_BG); b1r.pack(fill="x", pady=(2, 6))
tk.Button(b1r, text="DEMARRER robot (bringup)", command=bringup_start,
          bg="#2f7d9d", fg="white", font=("", 10, "bold"), bd=0,
          cursor="hand2", activebackground="#3a95bb").pack(
    side="left", expand=True, fill="x", ipady=5, padx=(0, 3))
tk.Button(b1r, text="Couper bringup", command=bringup_stop,
          bg="#5a4a6a", fg="white", bd=0, cursor="hand2",
          activebackground="#6a5a7a").pack(side="left", ipady=5, padx=(3, 0))

# Couche 2 : comportement
tk.Label(af, text="2. COMPORTEMENT (sur bringup actif)", bg=C_BG, fg=C_SUB,
         font=("", 8, "bold")).pack(anchor="w")
b2r = tk.Frame(af, bg=C_BG); b2r.pack(fill="x", pady=(2, 0))
launch_btn = tk.Button(b2r, text="LANCER MODE", command=behavior_start,
                       bg=C_ACCENT, fg="#10202a", font=("", 11, "bold"),
                       bd=0, cursor="hand2", activebackground="#3adfe2")
launch_btn.pack(side="left", expand=True, fill="x", ipady=6, padx=(0, 3))
tk.Button(b2r, text="Couper mode", command=lambda: behavior_stop(),
          bg="#5a5a3a", fg="white", bd=0, cursor="hand2",
          activebackground="#6a6a4a").pack(side="left", ipady=6, padx=(3, 0))

stop_btn = tk.Button(af, text="STOP — TOUT TUER (bringup + mode)",
                     command=stop_everything, bg=C_DANGER, fg="white",
                     font=("", 11, "bold"), bd=0, cursor="hand2",
                     activebackground="#f06b6b")
stop_btn.pack(fill="x", ipady=6, pady=(8, 0))

dsf = card(root, "DONNEES  (video + lidar sur les robots)")
d1 = tk.Frame(dsf, bg=C_CARD); d1.pack(fill="x")
tk.Button(d1, text="PULL  (recuperer + liberer)", command=ds_pull,
          bg=C_GREEN, fg="white", bd=0, cursor="hand2",
          activebackground="#39985f", font=("", 9, "bold")).pack(
    side="left", expand=True, fill="x", ipady=3, padx=(0, 3))
tk.Button(d1, text="Supprimer", command=ds_clean, bg="#8a3a3a", fg="white",
          bd=0, cursor="hand2", activebackground="#a84848",
          font=("", 9)).pack(side="left", expand=True, fill="x", ipady=3,
                             padx=(3, 0))
d2 = tk.Frame(dsf, bg=C_CARD); d2.pack(fill="x", pady=(4, 0))
sbtn(d2, "Copier sans supprimer", ds_collect).pack(side="left", expand=True,
                                                   fill="x", padx=(0, 3))
sbtn(d2, "Espace disque", ping_space_async).pack(side="left", expand=True,
                                                 fill="x", padx=(3, 0))

devf = card(root, "DEV — Build & Deploy")
sbtn(devf, "⚙ Build + Deploy (1-2 min)", build_deploy,
     bg="#3a3a4a").pack(fill="x")

jf = card(root, "MONITORING  (journal horodate)")
journal = tk.Text(jf, height=7, bg="#0e0e15", fg="#b8f0c8", bd=0,
                  font=("Courier", 8), state="disabled", wrap="word")
journal.pack(fill="both", expand=True)

status = tk.StringVar(value="Pret.")
tk.Label(root, textvariable=status, bg=C_BG, fg=C_SUB,
         wraplength=530).pack(side="bottom", pady=5)

view_windows = {}


def open_view(idx):
    want_cam = cam_show[idx].get()
    want_lid = lidar_show[idx].get()
    if not want_cam and not want_lid:
        if idx in view_windows:
            view_windows[idx][0].destroy()
            del view_windows[idx]
        return
    if idx not in view_windows:
        win = tk.Toplevel(root)
        win.title(f"tortuga{idx} — retour capteurs")
        win.configure(bg=C_BG)
        def on_close(i=idx):
            cam_show[i].set(False); lidar_show[i].set(False)
            view_windows[i][0].destroy(); del view_windows[i]
        win.protocol("WM_DELETE_WINDOW", on_close)
        cam_cv = tk.Canvas(win, width=320, height=240, bg="#0c0c12",
                           highlightthickness=0)
        lid_cv = tk.Canvas(win, width=240, height=240, bg="#0c0c12",
                           highlightthickness=0)
        info_lbl = tk.Label(win, text="", bg=C_BG, fg=C_TXT,
                            font=("Courier", 9))
        cam_cv.grid(row=0, column=0, padx=4, pady=4)
        lid_cv.grid(row=0, column=1, padx=4, pady=4)
        info_lbl.grid(row=1, column=0, columnspan=2)
        view_windows[idx] = [win, cam_cv, lid_cv, None, info_lbl]
    _, cam_cv, lid_cv, _, _ = view_windows[idx]
    cam_cv.grid() if want_cam else cam_cv.grid_remove()
    lid_cv.grid() if want_lid else lid_cv.grid_remove()


def draw_lidar(canvas, msg):
    canvas.delete("all")
    if not msg.ranges or msg.angle_increment == 0.0:
        canvas.create_text(120, 120, text="scan vide", fill="#e05555")
        return
    cx, cy, sc = 120, 120, 55.0
    for rad, col in ((1.0, "#2a2a3c"), (2.0, "#2a2a3c"), (SAFETY, "#a03030")):
        r = rad * sc
        canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline=col)
    canvas.create_text(cx+SAFETY*sc+2, cy, text=f"{SAFETY:.2f}", fill="#a03030",
                       anchor="w", font=("", 7))
    canvas.create_line(cx, cy, cx, cy-22, fill=C_ACCENT)
    r = np.array(msg.ranges)
    ok = np.isfinite(r) & (r > 0.06) & (r < 2.2)
    ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    for a, d in zip(ang[ok], r[ok]):
        x = cx - math.sin(a) * d * sc
        y = cy - math.cos(a) * d * sc
        col = C_BAD if d < SAFETY + 0.05 else "#7fd1d3"
        canvas.create_rectangle(x, y, x+2, y+2, fill=col, outline="")
    canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill=C_TXT, outline="")


def update_state_lights():
    if hub is None:
        return
    for i in ROBOTS:
        tp = hub.topics_by_robot.get(i, set())
        base = f"/tortuga{i}/"
        motors = f"{base}odom" in tp or f"{base}cmd_vel" in tp
        lidar = f"{base}scan" in tp
        cam = any(t.startswith(f"{base}camera") for t in tp)
        wander = any(t.endswith("wander") or "wander" in t for t in tp)
        for key, on in (("M", motors), ("L", lidar), ("C", cam)):
            if i in state_lbl:
                state_lbl[i][key].config(
                    fg=(C_OK if on else "#666"),
                    bg=("#1e3a25" if on else "#333"))
        # mode actif : deduit des noeuds de comportement presents
        mode = "--"
        if any("tracker" in t for t in tp):
            mode = "cascade"
        elif any("recorder" in t for t in tp):
            mode = "dataset"
        elif motors and lidar:
            mode = "pret"
        if i in mode_lbl:
            mode_lbl[i].config(text=mode,
                               fg=(C_ACCENT if mode not in ("--", "pret") else C_SUB))


def views_tick():
    update_state_lights()
    if ROS_OK and hub is not None and hub.node is not None:
        for idx, entry in list(view_windows.items()):
            win, cam_cv, lid_cv, _, info_lbl = entry
            txt = []
            if cam_show[idx].get():
                f = hub.frames.get(idx)
                if f is not None:
                    fr = detect_boxes(f.copy())
                    fr = cv2.resize(fr, (320, 240))
                    fr = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
                    ok, buf = cv2.imencode(".ppm", fr)
                    if ok:
                        img = tk.PhotoImage(data=buf.tobytes())
                        entry[3] = img
                        cam_cv.delete("all")
                        cam_cv.create_image(0, 0, anchor="nw", image=img)
                    txt.append("cam OK")
                else:
                    txt.append("cam: pas de flux")
            if lidar_show[idx].get():
                s = hub.scans.get(idx)
                if s is not None:
                    draw_lidar(lid_cv, s)
                    av = sector_min(s, 0, 20)
                    fav = " --- " if av >= 90 else f"{av:4.2f}m"
                    warn = "  OBSTACLE" if av < SAFETY + 0.05 else ""
                    txt.append(f"avant {fav}{warn}")
                else:
                    txt.append("lidar: pas de flux")
            info_lbl.config(text="   ".join(txt))
    root.after(150, views_tick)


hub = Hub() if ROS_OK else None
refresh_info()
if not ROS_OK:
    log("ROS non source : cam/lidar/map inactifs. 'source' puis relance.")


def deferred_start():
    # Tout ce qui touche au reseau/ROS demarre APRES le 1er affichage,
    # pour que la fenetre apparaisse immediatement.
    if hub is not None:
        hub.start()
    log("Demarrage termine. Rafraichis l'etat si besoin.")
    ping_space_async()
    root.after(500, views_tick)


log("Interface prete.")
root.after(400, deferred_start)
root.update_idletasks()
root.mainloop()
