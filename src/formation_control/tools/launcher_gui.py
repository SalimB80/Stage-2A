#!/usr/bin/env python3
"""
Interface de lancement TurtleBot3 : modes FORMATION et TRACKER.

Arret robuste :
  - Bouton "TOUT ARRETER" : tue TOUS les noeuds (bringup, camera, lidar,
    follower, tracker) sur TOUS les robots, en tache de fond, sans jamais
    attendre. Couper le noeud moteur stoppe le mouvement automatiquement
    (le TurtleBot arrete les roues quand plus personne ne publie cmd_vel).
  - Rien dans la GUI n'utilise 'ros2 topic pub' (qui bloque sur 'waiting
    subscription'). Tous les appels sont non-bloquants (Popen detache).
"""

import tkinter as tk
from tkinter import ttk
import subprocess

ROBOTS = {
    1: ("tortuga1", "192.168.0.201"),
    2: ("tortuga2", "192.168.0.202"),
    3: ("tortuga3", "192.168.0.203"),
    4: ("tortuga4", "192.168.0.204"),
}
FORMATIONS = ["colonne", "ligne", "triangle", "carre"]
PW = "1234"
ROS_DOMAIN_ID = 30

# Motifs de tous les process a tuer sur les robots.
KILL_PATTERNS = ("camera_node", "v4l2_camera_node", "turtlebot3_ros",
                 "single_coin_d4", "robot_state_publisher", "diff_drive",
                 "follower", "tracker", "robot_full.launch", "ros2 launch")


def robot_env():
    return (
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
        "export TURTLEBOT3_MODEL=burger; "
        "export LDS_MODEL=LDS-03; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/turtlebot3_ws/install/setup.bash; "
        "source ~/formation_ws/install/setup.bash; "
    )


def pc_env():
    return (
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/formation_ws/install/setup.bash; "
    )


def ssh_bg(idx, cmd):
    """SSH non-bloquant (detache), ne fige jamais la GUI."""
    user, ip = ROBOTS[idx]
    subprocess.Popen(
        ["sshpass", "-p", PW, "ssh",
         "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=5",
         "-o", "BatchMode=no", f"{user}@{ip}", cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ssh_launch(idx, ros_cmd):
    ssh_bg(idx, robot_env() + ros_cmd)


def ping_ok(ip):
    return subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def present_indices():
    return sorted(i for i in ROBOTS if present_vars[i].get())


def resolve_leader(present):
    forced = leader_var.get()
    if forced != "auto" and int(forced) in present:
        return int(forced)
    return present[0] if present else None


def refresh_info(*_):
    present = present_indices()
    if mode_var.get() == "tracker":
        info.set("Mode TRACKER : "
                 + (", ".join(f"tortuga{i}" for i in present) or "aucun")
                 + " autonomes")
        return
    leader = resolve_leader(present)
    if leader is None:
        info.set("Aucun robot present")
    else:
        foll = [i for i in present if i != leader]
        info.set(f"Leader tortuga{leader} | Followers "
                 + (", ".join('tortuga'+str(i) for i in foll) or 'aucun'))


def check_presence():
    present = present_indices()
    if not present:
        status.set("Coche au moins un robot.")
        return
    bad = [i for i in present if not ping_ok(ROBOTS[i][1])]
    status.set(("Injoignables : " + ", ".join(f"tortuga{i}" for i in bad))
               if bad else
               "Tous joignables : " + ", ".join(f"tortuga{i}" for i in present))


def launch():
    present = present_indices()
    if not present:
        status.set("Coche au moins un robot.")
        return
    if mode_var.get() == "tracker":
        for i in present:
            ssh_launch(i, f"ros2 launch formation_control robot_full.launch.py "
                          f"namespace:=tortuga{i} robot_index:={i} role:=tracker")
        status.set("Trackers lances : " + ", ".join(f"tortuga{i}" for i in present))
        return
    leader = resolve_leader(present)
    formation = form_var.get()
    for i in present:
        role = "leader" if i == leader else "follower"
        ssh_launch(i, f"ros2 launch formation_control robot_full.launch.py "
                      f"namespace:=tortuga{i} robot_index:={i} "
                      f"formation:={formation} role:={role}")
    foll = [i for i in present if i != leader]
    status.set(f"Leader tortuga{leader}, followers {foll or 'aucun'} - {formation}")


def change_formation():
    if mode_var.get() == "tracker":
        status.set("Inutile en mode tracker.")
        return
    present = present_indices()
    leader = resolve_leader(present)
    formation = form_var.get()
    for i in present:
        if i == leader:
            continue
        subprocess.Popen(["bash", "-c",
            pc_env() + f"ros2 param set /tortuga{i}/follower formation {formation}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    status.set(f"Formation -> {formation}")


def teleop_leader():
    if mode_var.get() == "tracker":
        status.set("Mode tracker : pas de leader.")
        return
    present = present_indices()
    leader = resolve_leader(present)
    if leader is None:
        status.set("Aucun leader.")
        return
    subprocess.Popen(["bash", "-c",
        pc_env() + f"ros2 run teleop_twist_keyboard teleop_twist_keyboard "
        f"--ros-args -r cmd_vel:=/tortuga{leader}/cmd_vel; exec bash"])
    status.set(f"Teleop tortuga{leader} (terminal ouvert)")


def stop_everything():
    """TOUT ARRETER : tue tous les noeuds sur tous les robots, non-bloquant.
    Couper le noeud moteur stoppe le mouvement automatiquement."""
    pk = "; ".join(f"pkill -9 -f '{p}'" for p in KILL_PATTERNS)
    # Deux passes rapprochees pour les process qui se relancent (launch parent).
    full = f"{pk}; sleep 0.5; {pk}"
    for i in ROBOTS:                      # TOUS les robots, coches ou non
        ssh_bg(i, full)
    status.set("ARRET envoye a tous les robots (noeuds tues, roues stoppees)")


# ---------------- UI ----------------
root = tk.Tk()
root.title("Formation / Tracker TurtleBot3")
root.geometry("400x540")

tk.Label(root, text="Mode", font=("", 11, "bold")).pack(pady=(10, 2))
mode_var = tk.StringVar(value="formation")
mode_var.trace_add("write", refresh_info)
mframe = tk.Frame(root); mframe.pack()
tk.Radiobutton(mframe, text="Formation (leader + followers)",
               variable=mode_var, value="formation").pack(anchor="w")
tk.Radiobutton(mframe, text="Tracker (autonome, cherche couleur)",
               variable=mode_var, value="tracker").pack(anchor="w")

tk.Label(root, text="Robots presents", font=("", 11, "bold")).pack(pady=(10, 2))
present_vars = {}
pframe = tk.Frame(root); pframe.pack()
for i in ROBOTS:
    v = tk.BooleanVar(value=False)
    present_vars[i] = v
    v.trace_add("write", refresh_info)
    tk.Checkbutton(pframe, text=f"tortuga{i} ({ROBOTS[i][1]})",
                   variable=v, anchor="w", width=26).pack(anchor="w")

tk.Label(root, text="Leader (mode formation)").pack(pady=(8, 2))
leader_var = tk.StringVar(value="auto")
leader_var.trace_add("write", refresh_info)
ttk.Combobox(root, textvariable=leader_var, values=["auto", "1", "2", "3", "4"],
             state="readonly", width=10).pack()

tk.Label(root, text="Formation").pack(pady=(8, 2))
form_var = tk.StringVar(value="colonne")
ttk.Combobox(root, textvariable=form_var, values=FORMATIONS,
             state="readonly", width=14).pack()

info = tk.StringVar(value="")
tk.Label(root, textvariable=info, fg="#0066aa", wraplength=360).pack(pady=6)

tk.Button(root, text="Verifier presence (ping)",
          command=check_presence).pack(pady=(6, 2), fill="x", padx=40)
tk.Button(root, text="Lancer", command=launch, bg="#00CED1",
          font=("", 12, "bold")).pack(pady=4, fill="x", padx=40)
tk.Button(root, text="Changer formation",
          command=change_formation).pack(pady=2, fill="x", padx=40)
tk.Button(root, text="Teleop leader (clavier)",
          command=teleop_leader).pack(pady=2, fill="x", padx=40)

tk.Button(root, text="TOUT ARRETER", command=stop_everything,
          bg="#c0392b", fg="white",
          font=("", 13, "bold")).pack(pady=(12, 4), fill="x", padx=30, ipady=6)

status = tk.StringVar(value="Pret")
tk.Label(root, textvariable=status, fg="gray", wraplength=360).pack(pady=8)

refresh_info()
root.mainloop()
