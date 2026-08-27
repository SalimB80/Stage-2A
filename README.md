# Stage-2A — TurtleBot3 fleet: formations, cascade following and dataset capture

ROS 2 (Humble) workspace for a fleet of four **TurtleBot3 Burger** robots. It
covers three things:

- **cascade following** — each robot tracks the coloured helmet of the robot in
  front of it, with a lidar relay so the chain survives a brief loss of vision;
- **autonomous wandering** — lidar-only obstacle avoidance, camera left free;
- **dataset capture** — synchronised camera + lidar + odometry recording, and the
  whole post-processing chain that turns it into videos and CSVs.

## Hardware and network

| Item | Value |
|---|---|
| Robots | 4x TurtleBot3 Burger, `tortugaX@192.168.0.20X` (X = 1..4) |
| Sensors | LDS lidar + Raspberry Pi camera on each robot |
| Robot OS | Ubuntu + ROS 2 Humble, workspace in `~/formation_ws` |
| Control PC | Windows + WSL2 |
| DDS domain | `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |

WSL2 sits behind a NAT, so the PC cannot reach the robots out of the box. Enable
mirrored networking in `C:\Users\<you>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown` from an admin PowerShell, restart WSL, and check
`ping 192.168.0.201`. If DDS multicast still does not get through, see
[`src/formation_control/config/README.md`](src/formation_control/config/README.md)
for the unicast profile.

## Repository layout

```
Stage-2A/
  src/formation_control/        the one and only ROS 2 package
    formation_control/          the ROS 2 nodes           -> README
    launch/                     launch files              -> README
    config/                     parameters + DDS profile  -> README
    scripts/                    ttb.sh fleet control      -> README
    tools/                      PC-side tooling and GUI   -> README
  rebuild.log                   leftover run log, not source
```

Each directory has its own README with the details:

- [`src/formation_control/`](src/formation_control/README.md) — the package itself
- [`src/formation_control/formation_control/`](src/formation_control/formation_control/README.md) — the nodes
- [`src/formation_control/launch/`](src/formation_control/launch/README.md) — the launch layers
- [`src/formation_control/config/`](src/formation_control/config/README.md) — parameters and DDS
- [`src/formation_control/scripts/`](src/formation_control/scripts/README.md) — `ttb.sh`
- [`src/formation_control/tools/`](src/formation_control/tools/README.md) — GUI, calibration, dataset pipeline

## First-time setup (PC only, once)

```bash
sudo apt install sshpass rsync tmux ffmpeg python3-tk
mkdir -p ~/formation_ws/src
# place this repository's src/formation_control in ~/formation_ws/src/
cd ~/formation_ws
colcon build --symlink-install
```

That is the only build you do by hand. **The robots are built from the GUI** —
its *Build + Deploy* button copies the package to each one and compiles it there.

## The three modes

The `mode` argument of `robot_behavior.launch.py` — and the vocabulary used
throughout the code — has three values. They are kept as-is in French because
they are parameter values:

**`errance` (wander).** `wander_node` drives the robot forward, meanders with a
slowly-varying random rotation, and avoids everything with the lidar alone
(walls, furniture, other robots). Body-aware avoidance: the robot is treated as
the circle circumscribing its 16x16 cm body, so its corners are covered while
turning. The camera is untouched, which is what makes this the base of the
dataset mode.

**`cascade`.** `tracker_node` follows the coloured helmet of the robot ahead:
tortuga1 wears yellow and is driven by hand, tortuga2 (red) follows the yellow
one, tortuga3 (green) follows the red one, tortuga4 (blue) follows the green one.
Colour identifies *who* to follow; as soon as a helmet is seen, a lock is armed
on that direction, and if the colour drops out the lidar takes over and tracks
the nearest robot-sized blob around the last known direction. The chain therefore
survives a blink of the vision instead of breaking.

**`dataset`.** `wander` + `recorder`. The recorder writes native JPEG frames (no
re-encoding, 55-60 fps) plus `frames.csv`, `scan.csv` and `odom.csv`, all
carrying both a wall-clock timestamp and the ROS stamp, so the streams are
directly alignable. It has a disk floor: it pauses below `min_free_mb` and
resumes once space is freed, so a Pi can never fill its card.

## How to start?

Open a terminal and run:

```bash
export DISPLAY=:0; export WAYLAND_DISPLAY=wayland-0; export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/formation_ws/install/setup.bash; python3 ~/formation_ws/src/formation_control/tools/launcher_gui.py
```

That is the whole entry point. **Everything is driven from the GUI** — deploying
to the robots, starting them, switching mode, driving the leader, recording and
collecting a dataset. There is nothing else to type in the terminal.

What the line does, in order: point the GUI at the display (`DISPLAY` /
`WAYLAND_DISPLAY`, needed under WSLg), join the fleet's DDS domain
(`ROS_DOMAIN_ID=30`), source ROS 2 and the workspace, then launch the console.

### Once the window is open

The left panel walks you through it, top to bottom:

**ROBOTS** — tick the robots you want to use. Battery level is shown next to
each one.

**1 · BUILD** — *Build + Deploy* copies the package onto the ticked robots and
builds it there. Needed the first time, and after every code change.

**2 · HARDWARE** — *Start robot* brings up motors, lidar and camera. Give it
~15 s; the camera fps re-locks itself just after (*Lock FPS* forces it by hand).
Leave this running — it is the layer everything else sits on.

**3 · MODE** — pick one, then *Start*:

- **Wander** — autonomous exploration, lidar-only avoidance.
- **Cascade** — driven leader + colour followers.
- **Dataset** — wander + video and lidar recording.

*Stop mode* stops the behaviour without touching the hardware layer, so you can
switch mode freely. Each robot row also has **ZQSD** to drive it from the
keyboard and **Halt** to stop it. **STOP — kill everything** is the panic button.

**Inspection tabs**, on the right: **Log**, **Camera** (raw / colour detection /
mask), **Lidar** (points / sectors / accumulated map), **Topics**,
**Formation** (per-robot colour and distance, plus *HSV Tuner…* for calibration),
and **Dataset** (*Pull data* to fetch a session, *Delete* to free the robots'
cards).

See [`tools/README.md`](src/formation_control/tools/README.md) for the details of
each panel.

> The terminal scripts (`ttb.sh`, `dataset_tools.sh`, …) still exist and do the
> same work — they are documented in
> [`scripts/README.md`](src/formation_control/scripts/README.md) and
> [`tools/README.md`](src/formation_control/tools/README.md) — but the GUI is the
> intended way in.

## Notes

- Always run the vision node **on** the robot. Streaming raw images to the PC
  over Wi-Fi does not work (14 MB/s); the compressed stream is what the tools
  subscribe to.
- Helmet colour thresholds shift with the room lighting. Calibrate with
  `tools/hsv_tuner.py` and copy the values into the `COLORS` dict in
  `formation_control/tracker_node.py` — a single place to edit.
- Comments across the codebase are in English. Log messages, GUI labels and
  parameter values are still in French; they are runtime strings and were left
  as they are.
