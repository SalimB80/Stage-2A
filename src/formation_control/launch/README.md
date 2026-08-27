# launch/ — launch files

Seven launch files in two generations. Read the namespace section below before
writing a new one — it is the trap this directory keeps running into.

| File | Generation | What it starts |
|---|---|---|
| `robot_bringup.launch.py` | **current**, layer 1 | TurtleBot3 bringup + camera |
| `robot_behavior.launch.py` | **current**, layer 2 | wander / tracker / recorder, by mode |
| `robot_dataset.launch.py` | all-in-one | bringup + camera + wander + recorder (+ SLAM) |
| `robot_full.launch.py` | all-in-one | bringup + camera + follower *or* tracker |
| `follower.launch.py` | legacy | the `follower` node alone |
| `leader.launch.py` | legacy | TurtleBot3 bringup alone |
| `robot_namespaced.launch.py` | legacy | bringup + camera under `PushRosNamespace` |

## The two-layer design (preferred)

The current approach splits hardware from behaviour, so you can change what a
robot is *doing* without restarting what it *is*.

**Layer 1 — `robot_bringup.launch.py`.** Motors, lidar and camera. Started once
and left up.

```bash
ros2 launch formation_control robot_bringup.launch.py namespace:=tortuga2
```

**Layer 2 — `robot_behavior.launch.py`.** Plugs into an already-running bringup.
It starts neither bringup nor camera, so it can be killed and relaunched freely
to switch mode.

```bash
ros2 launch formation_control robot_behavior.launch.py \
    namespace:=tortuga2 mode:=cascade role:=tracker target_color:=rouge
```

| `mode` | `role` | Nodes started |
|---|---|---|
| `errance` | — | `wander` |
| `dataset` | — | `wander` + `recorder` |
| `cascade` | `tracker` | `tracker` |
| `cascade` | `leader` | nothing — the leader is driven by hand |

### Arguments

`robot_bringup.launch.py`

| Argument | Default |
|---|---|
| `namespace` | `tortuga1` |

`robot_behavior.launch.py`

| Argument | Default | Notes |
|---|---|---|
| `namespace` | `tortuga1` | |
| `mode` | `errance` | `errance` / `cascade` / `dataset` |
| `role` | `tracker` | `leader` / `tracker`, cascade only |
| `target_color` | `jaune` | tracker only |
| `desired_bearing` | `0.0` | **must be a float**, see below |
| `target_distance` | `0.6` | tracker only |
| `robot_index`, `record` | `1`, `false` | declared, currently unused by the nodes |

> `desired_bearing` and `target_distance` are declared as doubles on the tracker.
> Passing `0` instead of `0.0` makes ROS 2 reject the parameter and the node
> crashes at startup.

Note that `robot_behavior.launch.py` defaults `target_distance` to `0.6`, while
`tracker_node.py` itself defaults to `0.32` and the GUI sends `0.32`. Pass it
explicitly if the distance matters.

## The namespace trap

**The TurtleBot3 bringup applies the namespace itself**, through its own
`namespace:=` argument. Wrapping that include in `PushRosNamespace` on top
applies it *twice*:

```
/tortuga2/tortuga2/scan     <- what you get
/tortuga2/scan              <- what every node subscribes to
```

Nothing errors. The nodes just sit there receiving nothing, exactly like a QoS
mismatch. The rule the current files follow:

- pass `namespace:=ns` to the bringup include — once;
- give **our** nodes the namespace through their own `namespace=` attribute, or
  wrap only them in a single `PushRosNamespace`;
- never both.

`robot_namespaced.launch.py` still has the old pattern (it wraps the bringup
include in `PushRosNamespace`). It is kept for reference; prefer
`robot_bringup.launch.py`.

## Camera settings

`robot_bringup.launch.py` and `robot_dataset.launch.py` start `camera_ros` at
640x480 with:

```python
'FrameDurationLimits': [18181, 18181]
```

libcamera has no "fps" setting — frame rate is the frame *duration*, in
microseconds. 18181 µs = 55 fps, above the 16971 µs hardware floor, so it is
valid. Locking min == max forces a fixed rate; it also pushes auto-exposure to a
short exposure, compensated by gain.

In practice an fps set at launch does not stick — auto-exposure takes back
control and the rate falls to ~18 fps. The GUI therefore re-locks the fps at
runtime, locally on each robot, ~16 s after bringup. See
[`../tools/README.md`](../tools/README.md) (`camera_fps.py`).

`~/image_raw` is remapped to `camera/image_raw`; the recorder subscribes to
`camera/image_raw/compressed`.

## All-in-one launch files

**`robot_dataset.launch.py`** — bringup + camera + `wander` + `recorder`, one
command per robot.

| Argument | Default |
|---|---|
| `namespace` | `tortuga1` |
| `record` | `true` |
| `slam` | `false` |

`slam:=true` adds `slam_toolbox` in async mapping mode, independent of `record`.
Despite the file name there is **no rosbag**: the recorder writes JPEGs plus
`scan.csv` and `odom.csv` itself, which is lighter on the Pi and directly usable.
The `.db3` bag was redundant (only the IMU was extra) and was removed.

**`robot_full.launch.py`** — bringup + camera + either `follower` or `tracker`,
selected by `role`.

| Argument | Default |
|---|---|
| `namespace` | `tortuga1` |
| `role` | `follower` (`follower` / `tracker`) |
| `robot_index`, `formation` | `1`, `colonne` (follower) |
| `target_color`, `desired_bearing`, `target_distance` | `jaune`, `0.0`, `0.6` (tracker) |

## Legacy files

- **`follower.launch.py`** — the `follower` node alone, with `robot_index`,
  `formation`, `namespace`. Assumes a bringup is already running.
- **`leader.launch.py`** — includes the TurtleBot3 bringup. It declares a
  `namespace` argument but never passes it to the include, so the bringup runs
  with its own default. Use `robot_bringup.launch.py` instead.
- **`robot_namespaced.launch.py`** — bringup + camera under a single
  `PushRosNamespace` around everything, i.e. the double-namespace pattern
  described above.

## What actually gets used

The GUI does not use these launch files for the behaviour layer. It runs the
nodes directly over SSH:

```bash
ros2 run formation_control tracker --ros-args -r __ns:=/tortuga2 \
    -p target_color:=rouge -p desired_bearing:=0.0 -p target_distance:=0.32
```

which is the validated path: a single namespace, sensor QoS on the scan, and the
process detached under `nohup` so an SSH hiccup cannot SIGHUP the node.
