# formation_control

ROS 2 (Humble) package for a fleet of TurtleBot3 Burger robots: formation
following, colour-cascade following, lidar-only wandering, and synchronised
dataset capture. The fleet adapts to 2, 3 or 4 robots.

## Contents

```
formation_control/
  formation_control/      ROS 2 nodes: tracker, wander, recorder, teleop, follower
  launch/                 launch files, two-layer bringup + behaviour
  config/                 follower parameters + Fast DDS unicast profile
  tools/                  PC-side tooling: GUI, calibration, dataset pipeline
  scripts/ttb.sh          single-command fleet control over SSH
  package.xml, setup.py   ament_python package definition
```

Each directory has its own README:
[nodes](formation_control/README.md) ·
[launch](launch/README.md) ·
[config](config/README.md) ·
[tools](tools/README.md) ·
[scripts](scripts/README.md)

## Hardware and context

- 4x TurtleBot3 Burger: `tortugaX@192.168.0.20X` (X = 1..4), password `1234`
- Lidar + Raspberry Pi camera on each robot
- Control PC on Windows + WSL, `ROS_DOMAIN_ID=30`

## Installation (WSL PC)

```bash
sudo apt install sshpass rsync tmux ffmpeg
mkdir -p ~/formation_ws/src
# place this folder in ~/formation_ws/src/formation_control
cd ~/formation_ws
colcon build --symlink-install
source install/setup.bash
chmod +x src/formation_control/scripts/ttb.sh
```

## WSL networking

WSL2 sits behind a NAT. Enable mirrored mode in `C:\Users\<you>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then `wsl --shutdown` (admin PowerShell) and restart WSL. Check with
`ping 192.168.0.201`.

If DDS multicast still does not get through, source the unicast profile
everywhere — on the PC *and* on every robot:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/formation_ws/src/formation_control/config/fastdds_unicast.xml
```

See [`config/README.md`](config/README.md).

## Two following implementations

The package holds two independent following paths. They share no code and are
started differently.

**Colour cascade — `tracker_node.py` (current).** Every robot wears a different
helmet colour and follows the one ahead of it: tortuga1 yellow (driven by hand),
tortuga2 red follows yellow, tortuga3 green follows red, tortuga4 blue follows
green. Colour identifies *who* to follow and gives the direction; the moment a
helmet is seen a lock is armed, and if the colour drops out the **lidar takes
over**, tracking the nearest robot-sized blob around the last known direction.
The chain survives a blink of the vision instead of breaking. Started by the GUI
or by `robot_behavior.launch.py mode:=cascade`.

**Single cyan helmet — `follower_node.py` (original).** One leader wearing a cyan
helmet; followers hold a per-formation (range, bearing) offset, bearing from the
camera and range fused from the lidar. Started by `ttb.sh start` and
`follower.launch.py`.

If you are changing following behaviour today, `tracker_node.py` is the file you
want. Details in [`formation_control/README.md`](formation_control/README.md).

## Fleet control: `ttb.sh`

One command for everything. `N` is the number of robots (default 4).

```bash
./ttb.sh deploy  [N]              # copy the package onto N robots
./ttb.sh start   [N] [formation]  # bringup + followers (background)
./ttb.sh teleop                   # drive the leader from the keyboard
./ttb.sh formation [N] [form]     # change formation at runtime
./ttb.sh monitor [N]              # tmux grid of the robot logs
./ttb.sh stop    [N]              # stop every node
```

### Typical workflow

```bash
./ttb.sh deploy 4               # once, and after every code change
./ttb.sh start 4 triangle       # start the fleet
./ttb.sh teleop                 # drive tortuga1; the others follow
./ttb.sh formation 4 carre      # switch formation without restarting
./ttb.sh stop 4                 # stop everything
```

With 2 or 3 robots, replace `4` with `2` or `3` everywhere.

Note that `deploy` copies but does **not** build — use `tools/deploy_build.sh` if
you want both. See [`scripts/README.md`](scripts/README.md).

## GUI

```bash
python3 tools/launcher_gui.py
```

Start the bringup, pick a mode (`wander` / `cascade` / `dataset`), and inspect
what is happening through the Log, Camera, Lidar, Topics and Dataset tabs. See
[`tools/README.md`](tools/README.md).

## Helmet colour calibration

Room lighting shifts the hue, so this has to be done on site.

**For the cascade** (`tracker_node.py`), on a robot camera over the network:

```bash
python3 tools/hsv_tuner.py tortuga3
```

Adjust until the helmet is cleanly isolated, press `P` to print the thresholds,
and copy them into the `COLORS` dict at the top of
`formation_control/tracker_node.py` — a single place to edit. Then redeploy.

**For the legacy cyan path** (`detector.py`), on a local webcam:

```bash
python3 tools/calibrate_hsv.py
```

Copy the values into `CYAN_LOW` / `CYAN_HIGH` in
`formation_control/detector.py`, then redeploy.

## Dataset capture

The dataset mode runs `wander` + `recorder` on each robot. The recorder writes,
per 5-minute segment, native JPEG frames (no re-encoding, 55-60 fps) plus
`frames.csv`, `scan.csv` and `odom.csv`, each row carrying a millisecond wall
clock *and* the ROS stamp from the same clock across all sensors — so frames,
scans and poses align exactly.

A full session:

```bash
cd tools
./dataset_tools.sh start 1 2 3 4     # wander + record
./dataset_tools.sh drain 2 3         # optional: pull + purge while recording
./dataset_tools.sh stop              # clean stop, recorder first
./dataset_tools.sh collect 1 2 3 4   # pull + assemble + join + tidy
```

`collect` pulls everything, assembles each segment into an mp4 using the real
frame timestamps, joins them into `<robot>_<session>_final.mp4`, converts any
rosbag to CSV, and tidies each session into a self-contained folder. The full
pipeline, and the individual tools behind it, are documented in
[`tools/README.md`](tools/README.md).

Current runs record **no rosbag**: the recorder writes the CSVs itself, which is
lighter on the Pi and directly usable. `tools/bag_to_csv.py` remains for bags
captured before that change.

## Available formations

`colonne`, `ligne`, `triangle`, `carre`. The per-follower (range, bearing)
offsets are defined in `formation_control/formations.py` and are easy to edit;
the table is reproduced in [`config/README.md`](config/README.md).

These apply to the `follower` path. In the cascade, geometry comes from each
tracker's `desired_bearing` and `target_distance` instead.

## Known limitations

- **Pure vision:** a follower has to see the helmet. Deep formations (a 4th robot
  hidden behind the others) are fragile. The lidar relay in `tracker_node.py`
  covers brief losses, not sustained occlusion.
- **The cyan path has one helmet, so one visible leader**, and followers cannot
  tell each other apart. For strict lateral formations, distinct AprilTags would
  be more robust than colour.
- **Run the vision node on the robot**, never stream raw images to the PC — the
  raw stream is ~14 MB/s and does not survive Wi-Fi.
- **An fps set at launch does not stick**; auto-exposure takes back control and
  the rate drops to ~18 fps. It has to be re-locked at runtime on the robot (the
  GUI does this automatically ~16 s after bringup).
- **Namespace applied twice** is the classic silent failure: the TurtleBot3
  bringup already applies it, so an extra `PushRosNamespace` yields
  `/tortugaX/tortugaX/scan` and every subscriber goes quiet with no error. See
  [`launch/README.md`](launch/README.md).

## Bring-up checklist, 0 → 100%

1. Colour calibration on a real image, under the room's lighting.
2. Detection only: bearing and range printed, robot stationary.
3. One follower tracking a leader pushed by hand (column).
4. Tune the `k_lin` / `k_ang` gains.
5. Two robots, then three, then four.
6. All formations, plus runtime transitions between them.
