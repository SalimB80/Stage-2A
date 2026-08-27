# scripts/ — fleet control from the terminal

Terminal-side control of the whole fleet over SSH. This is the no-GUI path; the
graphical equivalent is [`../tools/launcher_gui.py`](../tools/README.md).

| File | What it is |
|---|---|
| `ttb.sh` | one command to deploy, start, drive, reconfigure, monitor and stop the fleet |
| `temporaire.py` | throwaway snippet that creates `~/formation_ws/src` on tortuga1..3 |

## Prerequisites

```bash
sudo apt install sshpass rsync tmux
chmod +x ttb.sh
```

The script assumes the fleet convention used everywhere in this repo:
`tortugaX@192.168.0.20X`, password `1234`, `ROS_DOMAIN_ID=30`,
`TURTLEBOT3_MODEL=burger`, and a workspace at `~/formation_ws` on each robot.
Those are hardcoded at the top of the file — change them there if the fleet
changes.

## `ttb.sh`

`N` is the number of robots, default 4. `formation` defaults to `colonne`.

```bash
./ttb.sh deploy  [N]              # rsync the package onto N robots
./ttb.sh start   [N] [formation]  # bringup + followers, in the background
./ttb.sh teleop                   # drive the leader from the keyboard
./ttb.sh formation [N] [form]     # change formation at runtime
./ttb.sh monitor [N]              # tiled tmux grid of every robot's log
./ttb.sh stop    [N]              # kill every node
```

Available formations: `colonne`, `ligne`, `triangle`, `carre` (see
[`../config/README.md`](../config/README.md)).

### Typical workflow

```bash
./ttb.sh deploy 4               # once, and after every code change
./ttb.sh start 4 triangle       # start the fleet
./ttb.sh teleop                 # drive tortuga1; the others follow
./ttb.sh formation 4 carre      # switch formation without restarting anything
./ttb.sh stop 4                 # stop everything
```

With 2 or 3 robots, just replace `4` with `2` or `3` everywhere.

### What each subcommand actually does

- **`deploy`** — `mkdir -p ~/formation_ws/src/formation_control` on each robot,
  then `rsync -az --delete` of the package into it.
- **`start`** — launches `turtlebot3_bringup robot.launch.py` with
  `namespace:=tortugaX` on every robot; on robots 2..N it then waits 3 s and
  starts the `follower` node with `robot_index` and `formation`. Everything runs
  detached under `nohup`, logging to `~/ros_<i>.log` on the robot.
- **`teleop`** — runs the standard `teleop_twist_keyboard` on the PC, remapped to
  `/tortuga1/cmd_vel`. (For the ZQSD layout and multi-topic auto-detection, use
  `ros2 run formation_control teleop_zqsd tortuga1` instead — see
  [`../formation_control/README.md`](../formation_control/README.md).)
- **`formation`** — `ros2 param set /tortugaX/follower formation <form>` on
  robots 2..N. The follower node re-reads its offset on the fly, so no restart.
- **`monitor`** — one tmux pane per robot, each running `tail -f ~/ros_<i>.log`.
- **`stop`** — `pkill -f ros2`, `pkill -f robot.launch`, `pkill -f follower`.

## Caveats

- **`deploy` does not build.** Its header says "copy + build", but the function
  only rsyncs; there is no `colcon build` in it. Either build on the robot
  afterwards, or use [`../tools/deploy_build.sh`](../tools/README.md), which
  copies *and* builds and reports a per-robot summary.
- **`ttb.sh` drives `follower`, not `tracker`.** It belongs to the original
  single-cyan-helmet path. The current colour cascade is started by
  `robot_behavior.launch.py` (`mode:=cascade`) or by the GUI. See
  [`../formation_control/README.md`](../formation_control/README.md) for which
  node is which.
- **`set -e` at the top** means one unreachable robot aborts the whole loop.
  `tools/deploy_build.sh` was written to fix exactly that for deployment.
- The SSH password is in clear text in the script, as are the robot IPs. Fine for
  an isolated lab network, not for anything else.

## `temporaire.py`

A one-off snippet that creates `~/formation_ws/src` on tortuga1..3. Despite the
`.py` extension its content is a bash `for` loop, so it only runs under `bash`,
not `python3`. Kept as-is; `ttb.sh deploy` and `deploy_build.sh` both create the
directory tree themselves, so it is not needed in normal use.
