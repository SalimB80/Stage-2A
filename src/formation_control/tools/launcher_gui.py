#!/usr/bin/env python3
"""
Launcher TurtleBot3 v4 — cascade multi-couleurs.

CASQUES fixes par robot : tortuga1=JAUNE, tortuga2=ROUGE, tortuga3=CYAN.
Mode CASCADE : la chaine se construit sur les robots coches, tries par
index. Le premier est leader (pilote, pas de noeud de suivi). Chaque
suivant traque LA COULEUR DU CASQUE du robot qui le precede.
Ex: 1+2+3 coches -> 2 traque 'jaune', 3 traque 'rouge'.
Ex: 2+3 coches   -> 2 leader, 3 traque 'rouge'.

La formation choisie regle le bearing de chaque maillon :
colonne = tout a 0 deg ; ligne/triangle = V aplati (angles alternes)
pour que chaque robot garde sa cible dans le champ de sa camera.
"""

import tkinter as tk
from tkinter import ttk
import subprocess
import threading

ROBOTS = {
    1: ("tortuga1", "192.168.0.201"),
    2: ("tortuga2", "192.168.0.202"),
    3: ("tortuga3", "192.168.0.203"),
    4: ("tortuga4", "192.168.0.204"),
}
# Couleur du casque PORTE par chaque robot (fixe, materiel).
HELMETS = {1: "jaune", 2: "rouge", 3: "cyan", 4: "jaune"}

# Bearing (deg) par position dans la chaine (maillon 2, 3, 4) et formation.
FORMATION_BEARINGS = {
    "colonne":  [0.0,   0.0,  0.0],
    "ligne":    [30.0, -30.0, 30.0],   # V aplati -> quasi-ligne
    "triangle": [25.0, -25.0, 0.0],
    "carre":    [20.0, -20.0, 0.0],
}
TARGET_DISTANCE = 0.6   # m, distance par maillon
PW = "1234"
ROS_DOMAIN_ID = 30

KILL_CMD = (
    "pkill -9 -f '[r]os2 launch'; "
    "pkill -9 -f '[r]obot_full.launch'; "
    "pkill -9 -f -- '[-]-ros-args'; "
    "sleep 0.7; "
    "pkill -9 -f -- '[-]-ros-args'; "
    "true"
)
CHECK_CMD = "pgrep -fc -- '[-]-ros-args' || true"

C_BG = "#1e1e2e"; C_CARD = "#2a2a3c"; C_TXT = "#e6e6f0"
C_SUB = "#a0a0b8"; C_ACCENT = "#00CED1"; C_DANGER = "#e05555"
C_OK = "#37c871"; C_BAD = "#e05555"; C_UNK = "#666677"
HELMET_HEX = {"jaune": "#e8d44d", "rouge": "#e05555", "cyan": "#4dd0e1"}


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


def open_terminal(bash_cmd, fallback_msg):
    for attempt in (
        ["wt.exe", "wsl.exe", "-e", "bash", "-lc", bash_cmd],
        ["cmd.exe", "/c", "start", "", "wsl.exe", "-e", "bash", "-lc", bash_cmd],
    ):
        try:
            subprocess.Popen(attempt, cwd="/mnt/c/",
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except FileNotFoundError:
            continue
    status.set(fallback_msg)
    return False


def present_indices():
    return sorted(i for i in ROBOTS if present_vars[i].get())


def build_chain():
    """Chaine cascade : [(robot, role, couleur_suivie, bearing), ...]"""
    chain = present_indices()
    if not chain:
        return []
    bearings = FORMATION_BEARINGS.get(form_var.get(), [0.0, 0.0, 0.0])
    out = [(chain[0], "leader", None, 0.0)]
    for k, rob in enumerate(chain[1:]):
        color = HELMETS[chain[k]]         # casque du robot precedent
        bear = bearings[k] if k < len(bearings) else 0.0
        out.append((rob, "tracker", color, bear))
    return out


def refresh_info(*_):
    chain = build_chain()
    if not chain:
        info.set("Coche les robots presents ci-dessus")
        return
    parts = [f"tortuga{chain[0][0]} = LEADER (casque {HELMETS[chain[0][0]]})"]
    for rob, _, color, bear in chain[1:]:
        parts.append(f"tortuga{rob} suit {color} ({bear:+.0f} deg)")
    info.set("  ->  ".join(parts))


def ping_all_async():
    status.set("Ping en cours...")
    def work():
        results = {}
        for i, (_, ip) in ROBOTS.items():
            ok = subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL).returncode == 0
            results[i] = ok
        root.after(0, lambda: apply_ping(results))
    threading.Thread(target=work, daemon=True).start()


def apply_ping(results):
    for i, ok in results.items():
        dots[i].itemconfig("dot", fill=C_OK if ok else C_BAD)
    up = [f"tortuga{i}" for i, ok in results.items() if ok]
    status.set("Joignables : " + (", ".join(up) or "aucun"))


def launch():
    chain = build_chain()
    if not chain:
        status.set("Coche au moins un robot.")
        return
    for rob, role, color, bear in chain:
        if role == "leader":
            ssh_bg(rob, robot_env() +
                   f"ros2 launch formation_control robot_full.launch.py "
                   f"namespace:=tortuga{rob} robot_index:={rob} role:=leader")
        else:
            ssh_bg(rob, robot_env() +
                   f"ros2 launch formation_control robot_full.launch.py "
                   f"namespace:=tortuga{rob} robot_index:={rob} role:=tracker "
                   f"target_color:={color} desired_bearing:={bear} "
                   f"target_distance:={TARGET_DISTANCE}")
    status.set(f"Cascade lancee ({form_var.get()}) — leader tortuga{chain[0][0]}")


def apply_formation():
    """Change les bearings a chaud sur les trackers deja lances."""
    chain = build_chain()
    for rob, role, color, bear in chain[1:]:
        subprocess.Popen(["bash", "-c", pc_env() +
            f"ros2 param set /tortuga{rob}/tracker desired_bearing {bear}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    status.set(f"Bearings appliques -> {form_var.get()}")


def teleop():
    chain = build_chain()
    if not chain:
        status.set("Aucun robot a piloter.")
        return
    target = chain[0][0]
    cmd = pc_env() + f"ros2 run formation_control teleop_zqsd tortuga{target}"
    if open_terminal(cmd, "Terminal indisponible — lance a la main : "
                          f"ros2 run formation_control teleop_zqsd tortuga{target}"):
        status.set(f"Teleop ZQSD ouvert pour tortuga{target}")


def stop_everything():
    status.set("ARRET : kill envoye, verification...")
    stop_btn.config(state="disabled")
    def work():
        procs = {i: subprocess.Popen(ssh_args(i, KILL_CMD, timeout=3),
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                 for i in ROBOTS}
        for p in procs.values():
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        report = []
        for i in ROBOTS:
            try:
                r = subprocess.run(ssh_args(i, CHECK_CMD, timeout=3),
                                   capture_output=True, text=True, timeout=8)
                out = (r.stdout or "").strip().splitlines()
                n = out[-1] if out else "0"
                if not n.isdigit():
                    n = "0"
                report.append(f"t{i}:" + ("propre" if n == "0" else f"{n}!"))
            except Exception:
                report.append(f"t{i}:off")
        def done():
            stop_btn.config(state="normal")
            status.set("ARRET — " + " | ".join(report))
        root.after(0, done)
    threading.Thread(target=work, daemon=True).start()


# ----------------------- UI -----------------------
root = tk.Tk()
root.title("TurtleBot3 — Cascade")
root.geometry("440x580")
root.configure(bg=C_BG)
style = ttk.Style(root)
style.theme_use("clam")
style.configure("TCombobox", fieldbackground=C_CARD, background=C_CARD,
                foreground=C_TXT)

def card(parent, title):
    f = tk.Frame(parent, bg=C_CARD)
    f.pack(fill="x", padx=14, pady=(10, 0))
    tk.Label(f, text=title, bg=C_CARD, fg=C_SUB,
             font=("", 9, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
    inner = tk.Frame(f, bg=C_CARD)
    inner.pack(fill="x", padx=12, pady=(0, 10))
    return inner

tk.Label(root, text="TurtleBot3 — Cascade multi-couleurs", bg=C_BG, fg=C_TXT,
         font=("", 14, "bold")).pack(pady=(14, 0))

rob_frame = card(root, "ROBOTS PRESENTS  (pastille = ping, carre = casque)")
present_vars, dots = {}, {}
for i in ROBOTS:
    row = tk.Frame(rob_frame, bg=C_CARD)
    row.pack(fill="x", pady=1)
    c = tk.Canvas(row, width=14, height=14, bg=C_CARD, highlightthickness=0)
    c.create_oval(3, 3, 12, 12, fill=C_UNK, outline="", tags="dot")
    c.pack(side="left", padx=(0, 4))
    dots[i] = c
    hc = tk.Canvas(row, width=14, height=14, bg=C_CARD, highlightthickness=0)
    hc.create_rectangle(3, 3, 12, 12, fill=HELMET_HEX.get(HELMETS[i], C_UNK),
                        outline="")
    hc.pack(side="left", padx=(0, 6))
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_info)
    tk.Checkbutton(row, text=f"tortuga{i}  (casque {HELMETS[i]})",
                   variable=v, bg=C_CARD, fg=C_TXT, selectcolor=C_BG,
                   activebackground=C_CARD, activeforeground=C_TXT,
                   anchor="w").pack(side="left")
    tk.Label(row, text=ROBOTS[i][1], bg=C_CARD, fg=C_SUB).pack(side="right")

tk.Button(rob_frame, text="Rafraichir l'etat (ping)", command=ping_all_async,
          bg=C_CARD, fg=C_SUB, bd=1, relief="solid",
          activebackground=C_BG, activeforeground=C_TXT).pack(fill="x", pady=(6, 0))

form_frame = card(root, "FORMATION (bearing des maillons)")
form_var = tk.StringVar(value="colonne")
form_var.trace_add("write", refresh_info)
ttk.Combobox(form_frame, textvariable=form_var,
             values=list(FORMATION_BEARINGS.keys()),
             state="readonly", width=14).pack(anchor="w")

info = tk.StringVar(value="")
tk.Label(root, textvariable=info, bg=C_BG, fg=C_ACCENT,
         wraplength=400, justify="left").pack(pady=(8, 0))

act = tk.Frame(root, bg=C_BG)
act.pack(fill="x", padx=14, pady=(10, 0))
tk.Button(act, text="LANCER LA CASCADE", command=launch, bg=C_ACCENT,
          fg="#10202a", font=("", 12, "bold"), bd=0, cursor="hand2",
          activebackground="#3adfe2").pack(fill="x", ipady=7)

row2 = tk.Frame(act, bg=C_BG); row2.pack(fill="x", pady=(6, 0))
tk.Button(row2, text="Teleop ZQSD (leader)", command=teleop, bg=C_CARD,
          fg=C_TXT, bd=0, cursor="hand2", activebackground="#3a3a52",
          activeforeground=C_TXT).pack(side="left", expand=True,
                                       fill="x", ipady=5, padx=(0, 3))
tk.Button(row2, text="Appliquer formation", command=apply_formation,
          bg=C_CARD, fg=C_TXT, bd=0, cursor="hand2",
          activebackground="#3a3a52",
          activeforeground=C_TXT).pack(side="left", expand=True,
                                       fill="x", ipady=5, padx=(3, 0))

stop_btn = tk.Button(act, text="STOP — TOUT TUER", command=stop_everything,
                     bg=C_DANGER, fg="white", font=("", 12, "bold"), bd=0,
                     cursor="hand2", activebackground="#f06b6b")
stop_btn.pack(fill="x", ipady=8, pady=(10, 0))

status = tk.StringVar(value="Pret. Rafraichis l'etat des robots.")
tk.Label(root, textvariable=status, bg=C_BG, fg=C_SUB,
         wraplength=400).pack(side="bottom", pady=10)

refresh_info()
ping_all_async()
root.mainloop()
