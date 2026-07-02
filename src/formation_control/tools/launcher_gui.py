#!/usr/bin/env python3
"""
Interface de lancement des formations TurtleBot3 (leader-follower).

- Coche les robots PRESENTS aujourd'hui.
- Le LEADER est automatiquement le plus petit index present
  (1 si present, sinon 2, sinon 3, sinon 4), ou forcable manuellement.
- Chaque robot present est bringup COMPLET (moteurs + lidar + odom + camera),
  le follower ne demarre que sur les non-leaders.
- Tout se lance par SSH depuis le PC (WSL), sans terminal manuel.
"""

import tkinter as tk
from tkinter import ttk
import subprocess

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ROBOTS = {
    1: ("tortuga1", "192.168.0.201"),
    2: ("tortuga2", "192.168.0.202"),
    3: ("tortuga3", "192.168.0.203"),
    4: ("tortuga4", "192.168.0.204"),
}

CAM_DEVICE = {  # device camera par robot, surcharge si besoin
    1: "/dev/video0",
    2: "/dev/video0",
    3: "/dev/video0",
    4: "/dev/video0",
}

FORMATIONS = ["colonne", "ligne", "triangle", "carre"]
PW = "1234"
ROS_DOMAIN_ID = 30

# Environnement source sur CHAQUE robot avant toute commande ROS.
# On source ROS puis le workspace ; on exporte le domain id et le modele.
def robot_env():
    return (
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
        "export TURTLEBOT3_MODEL=burger; "
        "export LDS_MODEL=LDS-03; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/turtlebot3_ws/install/setup.bash; "
        "source ~/formation_ws/install/setup.bash; "
    )

# Environnement local (PC) pour teleop.
def pc_env():
    return (
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/formation_ws/install/setup.bash; "
    )


# ---------------------------------------------------------------------------
# SSH
# ---------------------------------------------------------------------------
def ssh(idx, cmd, hold=False):
    """Execute une commande sur le robot idx via SSH (arriere-plan)."""
    user, ip = ROBOTS[idx]
    full = robot_env() + cmd
    if hold:
        full += "; exec bash"
    subprocess.Popen([
        "sshpass", "-p", PW, "ssh",
        "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=5",
        f"{user}@{ip}", full
    ])


def ping_ok(ip):
    """Verifie qu'un robot repond (1 ping, timeout 1s)."""
    r = subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


# ---------------------------------------------------------------------------
# Logique leader / followers
# ---------------------------------------------------------------------------
def present_indices():
    """Liste triee des index coches comme presents."""
    return sorted(i for i in ROBOTS if present_vars[i].get())


def resolve_leader(present):
    """Leader = choix manuel s'il est present, sinon plus petit index present."""
    forced = leader_var.get()
    if forced != "auto" and int(forced) in present:
        return int(forced)
    return present[0] if present else None


def refresh_leader_label(*_):
    present = present_indices()
    leader = resolve_leader(present)
    if leader is None:
        leader_info.set("Aucun robot present")
    else:
        followers = [i for i in present if i != leader]
        leader_info.set(
            f"Leader : tortuga{leader}   |   "
            f"Followers : {', '.join('tortuga'+str(i) for i in followers) or 'aucun'}")


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
def check_presence():
    """Ping les robots coches et signale ceux injoignables."""
    present = present_indices()
    if not present:
        status.set("Coche au moins un robot.")
        return
    unreachable = []
    for i in present:
        _, ip = ROBOTS[i]
        if not ping_ok(ip):
            unreachable.append(i)
    if unreachable:
        status.set("Injoignables : "
                   + ", ".join(f"tortuga{i}" for i in unreachable))
    else:
        status.set(f"Tous joignables : "
                   + ", ".join(f"tortuga{i}" for i in present))


def launch():
    present = present_indices()
    if not present:
        status.set("Coche au moins un robot avant de lancer.")
        return
    leader = resolve_leader(present)
    formation = form_var.get()

    for i in present:
        role = "leader" if i == leader else "follower"
        dev = CAM_DEVICE.get(i, "/dev/video0")
        ssh(i, f"ros2 launch formation_control "
               f"robot_full.launch.py namespace:=tortuga{i} "
               f"robot_index:={i} formation:={formation} role:={role}")

    followers = [i for i in present if i != leader]
    status.set(f"Lance : leader tortuga{leader}, "
               f"followers {followers or 'aucun'} — formation {formation}")


def change_formation():
    present = present_indices()
    leader = resolve_leader(present)
    formation = form_var.get()
    for i in present:
        if i == leader:
            continue
        subprocess.Popen(["bash", "-c",
            pc_env() + f"ros2 param set /tortuga{i}/follower formation {formation}"])
    status.set(f"Formation changee -> {formation}")


def teleop_leader():
    present = present_indices()
    leader = resolve_leader(present)
    if leader is None:
        status.set("Aucun leader a piloter.")
        return
    subprocess.Popen(["bash", "-c",
        pc_env() + f"ros2 run teleop_twist_keyboard teleop_twist_keyboard "
        f"--ros-args -r cmd_vel:=/tortuga{leader}/cmd_vel; exec bash"])
    status.set(f"Teleop leader tortuga{leader} lance (voir le terminal)")


def stop_all():
    for i in present_indices():
        subprocess.Popen(["bash", "-c",
            pc_env() + f"ros2 topic pub --once /tortuga{i}/cmd_vel "
            f"geometry_msgs/msg/Twist '{{}}'"])
    status.set("STOP envoye a tous les robots presents")


# ---------------------------------------------------------------------------
# Interface
# ---------------------------------------------------------------------------
root = tk.Tk()
root.title("Formation TurtleBot3")
root.geometry("380x460")

tk.Label(root, text="Robots presents", font=("", 11, "bold")).pack(pady=(10, 2))
present_vars = {}
frame = tk.Frame(root)
frame.pack()
for i in ROBOTS:
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_leader_label)
    tk.Checkbutton(frame, text=f"tortuga{i}  ({ROBOTS[i][1]})",
                   variable=v, anchor="w", width=24).pack(anchor="w")

tk.Label(root, text="Leader").pack(pady=(10, 2))
leader_var = tk.StringVar(value="auto")
leader_var.trace_add("write", refresh_leader_label)
ttk.Combobox(root, textvariable=leader_var,
             values=["auto", "1", "2", "3", "4"],
             state="readonly", width=10).pack()

leader_info = tk.StringVar(value="Aucun robot present")
tk.Label(root, textvariable=leader_info, fg="#0066aa").pack(pady=4)

tk.Label(root, text="Formation").pack(pady=(6, 2))
form_var = tk.StringVar(value="colonne")
ttk.Combobox(root, textvariable=form_var, values=FORMATIONS,
             state="readonly", width=14).pack()

tk.Button(root, text="Verifier presence (ping)",
          command=check_presence).pack(pady=(10, 2), fill="x", padx=40)
tk.Button(root, text="Lancer", command=launch, bg="#00CED1",
          font=("", 12, "bold")).pack(pady=4, fill="x", padx=40)
tk.Button(root, text="Changer formation",
          command=change_formation).pack(pady=2, fill="x", padx=40)
tk.Button(root, text="Teleop leader",
          command=teleop_leader).pack(pady=2, fill="x", padx=40)
tk.Button(root, text="STOP", command=stop_all, bg="#E74C3C",
          fg="white").pack(pady=6, fill="x", padx=40)

status = tk.StringVar(value="Pret")
tk.Label(root, textvariable=status, fg="gray", wraplength=340).pack(pady=8)

refresh_leader_label()
root.mainloop()
