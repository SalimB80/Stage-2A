#!/usr/bin/env python3
"""
TurtleBot3 Control — v8  (interface type Notion : sobre, claire, fiable)

Architecture 2 couches :
  1) BRINGUP  : bringup natif TurtleBot3 (moteurs+lidar) + camera.
  2) MODE     : errance | cascade | dataset, lance PAR-DESSUS le bringup.
Toutes les commandes utilisent  ros2 run ... --ros-args -r __ns:=/tortugaX
(la methode validee : un seul namespace, QoS capteur pour le scan).

Debug organise en 3 onglets : Journal | Topics | Noeuds.
Camera affichee 100% BRUTE (aucune conversion de couleur).
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

# ---------------- Config flotte ----------------
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
DATASET_SH = "./src/formation_control/tools/dataset_tools.sh"

# ---------------- Palette Notion ----------------
# Fond clair, gris doux, texte sombre, un seul accent (bleu ardoise).
C_BG = "#ffffff"          # fond principal
C_PANE = "#f7f7f5"        # panneaux (gris tres clair facon Notion)
C_PANE2 = "#efefec"       # zones enfoncees
C_LINE = "#e3e3e0"        # bordures fines
C_TXT = "#37352f"         # texte principal (presque noir chaud)
C_SUB = "#787774"         # texte secondaire gris
C_FAINT = "#9b9a97"       # texte tres discret
C_ACCENT = "#2f6fb3"      # bleu ardoise (accent unique)
C_ACCENT_BG = "#eaf1f8"   # fond accent leger
C_OK = "#448361"          # vert sobre
C_WARN = "#d9730d"        # orange sobre
C_ERR = "#e03e3e"         # rouge sobre
C_OKBG = "#edf5f0"
C_WARNBG = "#faf3ec"
C_ERRBG = "#fbecec"
HELMET_HEX = {"jaune": "#dfab01", "rouge": "#e03e3e", "cyan": "#2fa8b3"}

FONT = "Segoe UI"
MONO = "Cascadia Mono"


# ---------------- Environnements & SSH ----------------
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


# ---------------- Hub ROS (topics, scan, image) ----------------
class Hub:
    def __init__(self):
        self.frames = {}          # idx -> image BRUTE (np array, telle que recue)
        self.scans = {}           # idx -> LaserScan
        self.topics_by_robot = {i: set() for i in ROBOTS}
        self.topic_types = {}     # nom -> type
        self.subbed = set()
        self.node = None
        self.started = False

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
        # Decodage BRUT : on garde exactement ce que la camera envoie,
        # AUCUNE conversion de couleur. cv2.imdecode rend du BGR ; on le
        # convertit une seule fois en RGB pour Tk (Tk attend du RGB),
        # sans jamais reordonner les canaux au-dela de ce strict besoin.
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is not None:
                self.frames[idx] = bgr    # stocke le BGR d'origine
        except Exception:
            pass


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


# ---------------- Selection & chaine ----------------
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


# ---------------- Journal (avec niveaux) ----------------
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
    launch_btn.config(text={"cascade": "Lancer la cascade",
                            "errance": "Lancer l'errance",
                            "dataset": "Lancer le dataset"}[m])
    form_combo.configure(state="readonly" if m == "cascade" else "disabled")
    present = present_indices()
    if not present:
        info.set("Sélectionne un ou plusieurs robots.")
        return
    if m == "cascade":
        ch = build_chain()
        parts = [f"t{ch[0][0]} · leader"] + \
                [f"t{r}→{c}" for r, _, c, b in ch[1:]]
        info.set("Chaîne :  " + "    ".join(parts))
    elif m == "errance":
        info.set("Errance autonome (sécurité 0,16 m) — "
                 + ", ".join(f"t{i}" for i in present))
    else:
        info.set("Dataset vidéo + lidar — " + ", ".join(f"t{i}" for i in present))


# ---------------- Etat (ping + disque) ----------------
def refresh_state():
    log("Actualisation de l'état…")
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
    log("État actualisé.", "ok")


# ---------------- Couche 1 : BRINGUP ----------------
def bringup_start():
    present = present_indices()
    if not present:
        log("Sélectionne au moins un robot.", "warn")
        return
    for i in present:
        ssh_bg(i, robot_env() +
               f"ros2 launch turtlebot3_bringup robot.launch.py "
               f"namespace:=tortuga{i}")
        ssh_bg(i, robot_env() +
               f"ros2 run camera_ros camera_node "
               f"--ros-args -r __ns:=/tortuga{i} -r __node:=camera "
               f"-p format:=BGR888 -p width:=640 -p height:=480 "
               f"-r ~/image_raw:=camera/image_raw")
    log("Bringup + caméra lancés : " + ", ".join(f"t{i}" for i in present)
        + " (≈15 s de démarrage).", "ok")


def bringup_stop():
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[r]obot.launch'; pkill -9 -f '[t]urtlebot3_ros'; "
                  "pkill -9 -f '[s]ingle_coin'; pkill -9 -f '[c]amera_node'; "
                  "pkill -9 -f '[r]obot_state'; pkill -9 -f '[d]iff_drive'; "
                  "pkill -9 -f '[l]d08'")
    log("Bringup coupé : " + ", ".join(f"t{i}" for i in present), "warn")


# ---------------- Couche 2 : MODE ----------------
def behavior_start():
    present = present_indices()
    if not present:
        log("Sélectionne au moins un robot.", "warn")
        return
    m = mode_var.get()
    form = form_var.get()
    chain = build_chain() if m == "cascade" else None
    launch_btn.config(state="disabled")
    log("Préparation… (arrêt de l'ancien mode)")

    def work():
        # 1) COUPER l'ancien mode et ATTENDRE la fin des kills.
        # Indispensable : lancer sans attendre creait une COURSE entre le
        # pkill (asynchrone) et le ros2 run (asynchrone). Selon le timing SSH
        # de chaque robot, le pkill pouvait arriver APRES le lancement et tuer
        # le noeud a peine demarre -> "un seul robot sur deux marche" et
        # "impossible de relancer apres Couper". On tue en bloquant, puis on
        # laisse le DDS retirer les anciens noeuds avant de relancer.
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
        time.sleep(0.8)  # laisse le DDS liberer /tortugaX/wander|tracker

        # 2) RELANCER le mode choisi.
        if m == "cascade":
            for rob, role, color, bear in chain:
                if role == "leader":
                    root.after(0, lambda r=rob:
                               log(f"t{r} = leader (à piloter en ZQSD)."))
                    continue
                ssh_bg(rob, robot_env() +
                       f"ros2 run formation_control tracker "
                       f"--ros-args -r __ns:=/tortuga{rob} "
                       f"-p target_color:={color} -p desired_bearing:={bear} "
                       f"-p target_distance:={TARGET_DISTANCE}")
            root.after(0, lambda: log(f"Cascade lancée ({form}).", "ok"))
        else:
            for i in present:
                ssh_bg(i, robot_env() +
                       f"ros2 run formation_control wander "
                       f"--ros-args -r __ns:=/tortuga{i}")
                if m == "dataset":
                    ssh_bg(i, robot_env() +
                           f"ros2 run formation_control recorder "
                           f"--ros-args -r __ns:=/tortuga{i} "
                           f"-p robot_name:=tortuga{i} -p segment_minutes:=5.0")
                    ssh_bg(i, robot_env() +
                           f"mkdir -p ~/dataset && ros2 bag record "
                           f"-o ~/dataset/bag_$(date +%Y%m%d_%H%M%S)_tortuga{i} "
                           f"/tortuga{i}/scan /tortuga{i}/odom /tortuga{i}/imu")
            msg = ("Dataset lancé (enregistrement !) : " if m == "dataset"
                   else "Errance lancée : ") + ", ".join(f"t{i}" for i in present)
            root.after(0, lambda msg=msg: log(msg, "ok"))
        root.after(0, lambda: launch_btn.config(state="normal"))

    threading.Thread(target=work, daemon=True).start()


def behavior_stop(silent=False):
    present = present_indices() or list(ROBOTS)
    for i in present:
        ssh_bg(i, "pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
                  "pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'")
    if not silent:
        log("Mode coupé (bringup conservé) : "
            + ", ".join(f"t{i}" for i in present), "warn")


def apply_formation():
    if mode_var.get() != "cascade":
        log("La formation ne s'applique qu'en cascade.", "warn")
        return
    for rob, _, _, bear in build_chain()[1:]:
        subprocess.Popen(["bash", "-c", pc_env() +
            f"ros2 param set /tortuga{rob}/tracker desired_bearing {bear}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Formation appliquée à chaud ({form_var.get()}).", "ok")


def teleop_robot(idx):
    if open_terminal(pc_env() +
                     f"ros2 run formation_control teleop_zqsd tortuga{idx}",
                     f"Manuel : teleop_zqsd tortuga{idx}"):
        log(f"Téléop ZQSD tortuga{idx} (nouvelle fenêtre).")


# ---------------- STOP total & build ----------------
KILL_CMD = ("pkill -TERM -f '[r]ecorder'; pkill -TERM -f '[b]ag record'; sleep 1; "
            "pkill -9 -f '[r]obot.launch'; pkill -9 -f '[t]urtlebot3_ros'; "
            "pkill -9 -f '[s]ingle_coin'; pkill -9 -f '[c]amera_node'; "
            "pkill -9 -f '[r]obot_state'; pkill -9 -f '[d]iff_drive'; "
            "pkill -9 -f '[l]d08'; pkill -9 -f '[w]ander'; pkill -9 -f '[t]racker'; "
            "pkill -9 -f '[f]ollower'; pkill -9 -f -- '[-]-ros-args'; sleep 0.5; "
            "pkill -9 -f -- '[-]-ros-args'; true")
CHECK_CMD = "pgrep -fc -- '[-]-ros-args' || true"


def stop_everything():
    log("STOP — extinction de tous les nœuds…", "warn")
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
                               log("Arrêt terminé — " + "   ".join(rep),
                                   "ok" if all("!" not in x for x in rep) else "warn")))
    threading.Thread(target=work, daemon=True).start()


def build_deploy():
    idxs = " ".join(map(str, present_indices())) or "1 2 3 4"
    log(f"Build + deploy sur {idxs}… (1–2 min)")
    def work():
        try:
            r = subprocess.run(
                ["bash", "-c",
                 "cd ~/formation_ws && colcon build --symlink-install && "
                 "source install/setup.bash && "
                 f"./src/formation_control/tools/deploy_build.sh {idxs}"],
                capture_output=True, text=True, timeout=240)
            if r.returncode == 0:
                root.after(0, lambda: log("Build + deploy réussi.", "ok"))
            else:
                out = (r.stdout + r.stderr)[-300:]
                root.after(0, lambda: log(f"Échec build : {out}", "err"))
        except subprocess.TimeoutExpired:
            root.after(0, lambda: log("Build : délai dépassé (>4 min).", "err"))
        except Exception as e:
            root.after(0, lambda: log(f"Build : exception {e}", "err"))
    threading.Thread(target=work, daemon=True).start()


# ---------------- Données ----------------
def ds_pull():
    idx = " ".join(map(str, targets()))
    open_terminal(pc_env() + f"cd ~/formation_ws && {DATASET_SH} pull {idx}; "
                  "echo; echo '=== Terminé. Entrée ==='; read",
                  "Manuel : dataset_tools.sh pull")
    log(f"Pull lancé (robots {idx}).")


def ds_clean():
    idx = " ".join(map(str, targets()))
    if not messagebox.askyesno("Supprimer les données",
            f"Supprimer TOUTES les données (vidéo + lidar) sur {idx} ?\n\n"
            "Vérifie d'abord le Pull."):
        return
    open_terminal(pc_env() + f"cd ~/formation_ws && echo oui | "
                  f"{DATASET_SH} clean {idx}; echo '=== Fait. Entrée ==='; read",
                  "Manuel : dataset_tools.sh clean")
    log(f"Suppression lancée (robots {idx}).", "warn")


# ============================================================
#                        INTERFACE
# ============================================================
root = tk.Tk()
root.title("TurtleBot3 Control")
root.geometry("980x680")
root.configure(bg=C_BG)
root.minsize(880, 600)

style = ttk.Style(root)
style.theme_use("clam")
style.configure("TNotebook", background=C_BG, borderwidth=0)
style.configure("TNotebook.Tab", background=C_PANE, foreground=C_SUB,
                padding=(14, 7), font=(FONT, 9), borderwidth=0)
style.map("TNotebook.Tab",
          background=[("selected", C_BG)],
          foreground=[("selected", C_TXT)])
style.configure("TCombobox", fieldbackground="#fff", background="#fff",
                foreground=C_TXT, arrowcolor=C_SUB, bordercolor=C_LINE)


def hsep(parent):
    tk.Frame(parent, bg=C_LINE, height=1).pack(fill="x", pady=8)


# ---- barre laterale gauche (controle) + zone droite (debug) ----
main = tk.Frame(root, bg=C_BG)
main.pack(fill="both", expand=True)

left = tk.Frame(main, bg=C_BG, width=430)
left.pack(side="left", fill="both", expand=False)
left.pack_propagate(False)

right = tk.Frame(main, bg=C_PANE, width=550)
right.pack(side="right", fill="both", expand=True)

# ================= COLONNE GAUCHE : CONTROLE =================
head = tk.Frame(left, bg=C_BG)
head.pack(fill="x", padx=20, pady=(16, 0))
tk.Label(head, text="TurtleBot3 Control", bg=C_BG, fg=C_TXT,
         font=(FONT, 16, "bold")).pack(anchor="w")
tk.Label(head, text="Formations · Errance · Dataset", bg=C_BG, fg=C_FAINT,
         font=(FONT, 9)).pack(anchor="w")

body = tk.Frame(left, bg=C_BG)
body.pack(fill="both", expand=True, padx=20, pady=10)

# --- Robots ---
tk.Label(body, text="ROBOTS", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", pady=(4, 4))

present_vars, dots, space_lbl = {}, {}, {}
state_dot, mode_lbl = {}, {}
for i in ROBOTS:
    row = tk.Frame(body, bg=C_BG)
    row.pack(fill="x", pady=1)
    # ping
    dots[i] = tk.Label(row, text="●", bg=C_BG, fg=C_FAINT, font=(FONT, 9))
    dots[i].pack(side="left")
    # casque
    tk.Label(row, text="■", bg=C_BG, fg=HELMET_HEX[HELMETS[i]],
             font=(FONT, 11)).pack(side="left", padx=(4, 2))
    # checkbox + nom
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_info)
    tk.Checkbutton(row, text=f"tortuga{i}", variable=v, bg=C_BG, fg=C_TXT,
                   selectcolor="#fff", activebackground=C_BG,
                   activeforeground=C_TXT, font=(FONT, 10),
                   anchor="w", width=10).pack(side="left")
    # voyants M L C (pastilles compactes)
    st = tk.Frame(row, bg=C_BG)
    st.pack(side="left", padx=4)
    state_dot[i] = {}
    for key in ("M", "L", "C"):
        lb = tk.Label(st, text=key, bg=C_PANE2, fg=C_FAINT,
                      font=(MONO, 8, "bold"), width=2, height=1)
        lb.pack(side="left", padx=1)
        state_dot[i][key] = lb
    # espace disque
    space_lbl[i] = tk.Label(row, text="—", bg=C_BG, fg=C_SUB,
                            font=(MONO, 8), width=5)
    space_lbl[i].pack(side="left", padx=(4, 0))
    # actions rapides
    tk.Button(row, text="ZQSD", command=lambda i=i: teleop_robot(i),
              bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 8), cursor="hand2",
              activebackground=C_PANE2, padx=6).pack(side="right")

tk.Button(body, text="Actualiser  ·  ping + espace disque", command=refresh_state,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, pady=5).pack(fill="x", pady=(8, 0))

hsep(body)

# --- Etape 1 : materiel ---
tk.Label(body, text="1 · MATÉRIEL", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
r1 = tk.Frame(body, bg=C_BG)
r1.pack(fill="x")
tk.Button(r1, text="Démarrer le robot", command=bringup_start,
          bg=C_ACCENT, fg="white", bd=0, font=(FONT, 10, "bold"),
          cursor="hand2", activebackground="#2a5f9a", pady=7).pack(
    side="left", expand=True, fill="x", padx=(0, 4))
tk.Button(r1, text="Couper", command=bringup_stop,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, padx=12).pack(side="left")

hsep(body)

# --- Etape 2 : mode ---
tk.Label(body, text="2 · MODE", bg=C_BG, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
mode_var = tk.StringVar(value="errance")
mode_var.trace_add("write", refresh_info)
for val, txt, desc in (
        ("errance", "Errance", "exploration autonome, évitement 0,16 m"),
        ("cascade", "Cascade", "leader piloté + suiveurs couleur"),
        ("dataset", "Dataset", "errance + enregistrement vidéo & lidar")):
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
form_var = tk.StringVar(value="colonne")
form_var.trace_add("write", refresh_info)
form_combo = ttk.Combobox(fr, textvariable=form_var,
                          values=list(FORMATION_BEARINGS.keys()),
                          state="readonly", width=10, font=(FONT, 9))
form_combo.pack(side="left", padx=6)
tk.Button(fr, text="Appliquer à chaud", command=apply_formation,
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 8), cursor="hand2",
          activebackground=C_PANE2, padx=8).pack(side="left")

info = tk.StringVar(value="")
tk.Label(body, textvariable=info, bg=C_ACCENT_BG, fg=C_ACCENT,
         font=(FONT, 9), anchor="w", justify="left", wraplength=380,
         padx=10, pady=6).pack(fill="x", pady=(8, 0))

r2 = tk.Frame(body, bg=C_BG)
r2.pack(fill="x", pady=(6, 0))
launch_btn = tk.Button(r2, text="Lancer l'errance", command=behavior_start,
                       bg=C_OK, fg="white", bd=0, font=(FONT, 10, "bold"),
                       cursor="hand2", activebackground="#3a6f52", pady=7)
launch_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
tk.Button(r2, text="Couper le mode", command=lambda: behavior_stop(),
          bg=C_PANE, fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, padx=12).pack(side="left")

# --- Bas de colonne : donnees + STOP + build ---
bottom = tk.Frame(left, bg=C_BG)
bottom.pack(fill="x", side="bottom", padx=20, pady=(0, 14))
hsep(bottom)
dr = tk.Frame(bottom, bg=C_BG)
dr.pack(fill="x", pady=(0, 6))
tk.Button(dr, text="Pull données", command=ds_pull, bg=C_PANE, fg=C_TXT,
          bd=0, font=(FONT, 9), cursor="hand2", activebackground=C_PANE2,
          pady=5).pack(side="left", expand=True, fill="x", padx=(0, 3))
tk.Button(dr, text="Supprimer", command=ds_clean, bg=C_PANE, fg=C_ERR,
          bd=0, font=(FONT, 9), cursor="hand2", activebackground=C_ERRBG,
          pady=5).pack(side="left", expand=True, fill="x", padx=(3, 3))
tk.Button(dr, text="Build+Deploy", command=build_deploy, bg=C_PANE,
          fg=C_SUB, bd=0, font=(FONT, 9), cursor="hand2",
          activebackground=C_PANE2, pady=5).pack(side="left", expand=True,
                                                 fill="x", padx=(3, 0))
stop_btn = tk.Button(bottom, text="STOP — tout arrêter", command=stop_everything,
                     bg=C_ERR, fg="white", bd=0, font=(FONT, 10, "bold"),
                     cursor="hand2", activebackground="#c43535", pady=8)
stop_btn.pack(fill="x")

# ================= COLONNE DROITE : DEBUG =================
tk.Label(right, text="INSPECTION", bg=C_PANE, fg=C_FAINT,
         font=(FONT, 8, "bold")).pack(anchor="w", padx=16, pady=(14, 6))

nb = ttk.Notebook(right)
nb.pack(fill="both", expand=True, padx=12, pady=(0, 12))

# --- Onglet Journal ---
tab_log = tk.Frame(nb, bg=C_BG)
nb.add(tab_log, text="  Journal  ")
journal = tk.Text(tab_log, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9),
                  state="disabled", wrap="word", padx=10, pady=8)
journal.pack(fill="both", expand=True)
journal.tag_configure("time", foreground=C_FAINT)
journal.tag_configure("info", foreground=C_TXT)
journal.tag_configure("ok", foreground=C_OK)
journal.tag_configure("warn", foreground=C_WARN)
journal.tag_configure("err", foreground=C_ERR)

# --- Onglet Caméra ---
tab_cam = tk.Frame(nb, bg=C_BG)
nb.add(tab_cam, text="  Caméra  ")
cam_top = tk.Frame(tab_cam, bg=C_BG)
cam_top.pack(fill="x", padx=10, pady=8)
tk.Label(cam_top, text="Robot", bg=C_BG, fg=C_SUB,
         font=(FONT, 9)).pack(side="left")
cam_robot = tk.IntVar(value=1)
for i in ROBOTS:
    tk.Radiobutton(cam_top, text=f"t{i}", variable=cam_robot, value=i,
                   bg=C_BG, fg=C_TXT, selectcolor="#fff",
                   activebackground=C_BG, font=(FONT, 9)).pack(side="left")
tk.Label(cam_top, text="— image brute, sans traitement", bg=C_BG,
         fg=C_FAINT, font=(FONT, 8)).pack(side="left", padx=6)
cam_canvas = tk.Canvas(tab_cam, bg="#000", highlightthickness=1,
                       highlightbackground=C_LINE, width=480, height=360)
cam_canvas.pack(padx=10, pady=(0, 10))
cam_info = tk.Label(tab_cam, text="", bg=C_BG, fg=C_SUB, font=(MONO, 9))
cam_info.pack()

# --- Onglet Lidar ---
tab_lid = tk.Frame(nb, bg=C_BG)
nb.add(tab_lid, text="  Lidar  ")
lid_top = tk.Frame(tab_lid, bg=C_BG)
lid_top.pack(fill="x", padx=10, pady=8)
tk.Label(lid_top, text="Robot", bg=C_BG, fg=C_SUB,
         font=(FONT, 9)).pack(side="left")
lid_robot = tk.IntVar(value=1)
for i in ROBOTS:
    tk.Radiobutton(lid_top, text=f"t{i}", variable=lid_robot, value=i,
                   bg=C_BG, fg=C_TXT, selectcolor="#fff",
                   activebackground=C_BG, font=(FONT, 9)).pack(side="left")
lid_canvas = tk.Canvas(tab_lid, bg="#fbfbfa", highlightthickness=1,
                       highlightbackground=C_LINE, width=340, height=340)
lid_canvas.pack(padx=10, pady=(0, 6))
lid_info = tk.Label(tab_lid, text="", bg=C_BG, fg=C_SUB, font=(MONO, 9))
lid_info.pack()

# --- Onglet Topics ---
tab_top = tk.Frame(nb, bg=C_BG)
nb.add(tab_top, text="  Topics  ")
topics_txt = tk.Text(tab_top, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9),
                     state="disabled", wrap="none", padx=10, pady=8)
topics_txt.pack(fill="both", expand=True)
topics_txt.tag_configure("head", foreground=C_ACCENT, font=(MONO, 9, "bold"))
topics_txt.tag_configure("dbl", foreground=C_ERR)

# --- Onglet Noeuds ---
tab_nodes = tk.Frame(nb, bg=C_BG)
nb.add(tab_nodes, text="  Nœuds  ")
nodes_txt = tk.Text(tab_nodes, bg=C_BG, fg=C_TXT, bd=0, font=(MONO, 9),
                    state="disabled", wrap="word", padx=10, pady=8)
nodes_txt.pack(fill="both", expand=True)
nodes_txt.tag_configure("on", foreground=C_OK)
nodes_txt.tag_configure("off", foreground=C_FAINT)

# barre d'etat
status = tk.StringVar(value="Prêt.")
tk.Label(root, textvariable=status, bg=C_PANE, fg=C_SUB, anchor="w",
         font=(FONT, 9), padx=14, pady=4).pack(fill="x", side="bottom")


# ---------------- Rendus dynamiques ----------------
def update_state_lights():
    if hub is None:
        return
    for i in ROBOTS:
        tp = hub.topics_by_robot.get(i, set())
        base = f"/tortuga{i}/"
        motors = f"{base}odom" in tp or f"{base}cmd_vel" in tp
        lidar = f"{base}scan" in tp
        cam = any(t.startswith(f"{base}camera") for t in tp)
        for key, on in (("M", motors), ("L", lidar), ("C", cam)):
            lb = state_dot[i][key]
            if on:
                lb.config(fg="white", bg=C_OK)
            else:
                lb.config(fg=C_FAINT, bg=C_PANE2)


def render_cam():
    if hub is None:
        return
    i = cam_robot.get()
    bgr = hub.frames.get(i)
    if bgr is None:
        cam_canvas.delete("all")
        cam_canvas.create_text(240, 180, text="En attente du flux caméra…",
                               fill="#888", font=(FONT, 10))
        cam_info.config(text="")
        return
    # AFFICHAGE BRUT : conversion BGR->RGB uniquement pour Tk (obligatoire),
    # aucun autre traitement, aucun filtre, aucune detection dessinee.
    h, w = bgr.shape[:2]
    scale = min(480 / w, 360 / h)
    small = cv2.resize(bgr, (int(w * scale), int(h * scale)))
    # PPM (P6) attend du RGB. imencode('.ppm') suppose du BGR en entree et
    # ecrit du RGB correctement -> on passe le BGR d'origine tel quel.
    ok, buf = cv2.imencode(".ppm", small)
    if ok:
        img = tk.PhotoImage(data=buf.tobytes())
        cam_canvas._img = img
        cam_canvas.delete("all")
        cam_canvas.create_image(240, 180, image=img)
    cam_info.config(text=f"tortuga{i}   {w}×{h}   flux brut")


def render_lidar():
    if hub is None:
        return
    i = lid_robot.get()
    lid_canvas.delete("all")
    msg = hub.scans.get(i)
    if msg is None or not msg.ranges or msg.angle_increment == 0.0:
        lid_canvas.create_text(170, 170, text="En attente du scan…",
                               fill="#999", font=(FONT, 10))
        lid_info.config(text="")
        return
    cx, cy, sc = 170, 170, 75.0
    for rad in (0.5, 1.0, 1.5):
        r = rad * sc
        lid_canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline=C_LINE)
    rs = SAFETY * sc
    lid_canvas.create_oval(cx-rs, cy-rs, cx+rs, cy+rs, outline=C_ERR)
    lid_canvas.create_text(cx+rs+4, cy, text=f"{SAFETY:.2f} m", fill=C_ERR,
                           anchor="w", font=(FONT, 7))
    lid_canvas.create_line(cx, cy, cx, cy-26, fill=C_ACCENT, width=2)
    r = np.array(msg.ranges)
    ok = np.isfinite(r) & (r > 0.06) & (r < 2.0)
    ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    for a, d in zip(ang[ok], r[ok]):
        x = cx - math.sin(a) * d * sc
        y = cy - math.cos(a) * d * sc
        col = C_ERR if d < SAFETY + 0.05 else C_ACCENT
        lid_canvas.create_oval(x-1, y-1, x+1, y+1, fill=col, outline="")
    lid_canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill=C_TXT, outline="")
    av = sector_min(msg, 0, 20)
    fav = "—" if av >= 90 else f"{av:.2f} m"
    warn = "   ⚠ obstacle" if av < SAFETY + 0.05 else ""
    lid_info.config(text=f"tortuga{i}   avant {fav}{warn}")


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
        topics_txt.insert("end", "Aucun topic détecté.\n"
                          "Démarre un robot, puis patiente ~3 s.\n", "info")
    topics_txt.configure(state="disabled")


def render_nodes():
    # Etat deduit des topics (leger, sans SSH) : quels comportements tournent.
    if hub is None:
        return
    nodes_txt.configure(state="normal")
    nodes_txt.delete("1.0", "end")
    for i in ROBOTS:
        tp = hub.topics_by_robot.get(i, set())
        base = f"/tortuga{i}/"
        checks = [
            ("bringup (moteurs)", f"{base}odom" in tp),
            ("lidar", f"{base}scan" in tp),
            ("caméra", any(t.startswith(f"{base}camera") for t in tp)),
        ]
        line_on = any(c[1] for c in checks)
        nodes_txt.insert("end", f"tortuga{i}   ",
                         "on" if line_on else "off")
        parts = []
        for label, on in checks:
            parts.append(("● " if on else "○ ") + label)
        nodes_txt.insert("end", "   ".join(parts) + "\n",
                         "on" if line_on else "off")
    nodes_txt.insert("end", "\n○ = absent    ● = actif (déduit des topics)\n",
                     "off")
    nodes_txt.configure(state="disabled")


def tick():
    try:
        update_state_lights()
        cur = nb.index(nb.select())
        # 0 Journal, 1 Camera, 2 Lidar, 3 Topics, 4 Noeuds
        if cur == 1:
            render_cam()
        elif cur == 2:
            render_lidar()
        elif cur == 3:
            render_topics()
        elif cur == 4:
            render_nodes()
    except Exception:
        pass
    root.after(200, tick)


def deferred_start():
    if hub is not None:
        hub.start()
    refresh_state()
    log("Interface prête.", "ok")


hub = Hub() if ROS_OK else None
refresh_info()
if not ROS_OK:
    log("ROS non sourcé : inspection caméra/lidar/topics inactive.", "warn")
root.after(300, deferred_start)
root.after(400, tick)
root.mainloop()
