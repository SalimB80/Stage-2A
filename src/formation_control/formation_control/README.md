# formation_control/ — the ROS 2 nodes

The Python package itself. Five executables are declared in `setup.py`, plus two
support modules that are imported rather than run.

| Module | Entry point | Role |
|---|---|---|
| `tracker_node.py` | `tracker` | colour cascade following, with lidar relay |
| `wander_node.py` | `wander` | lidar-only random wandering |
| `recorder_node.py` | `recorder` | synchronised camera + lidar + odom capture |
| `teleop_zqsd.py` | `teleop_zqsd` | ZQSD keyboard teleop |
| `follower_node.py` | `follower` | original leader-follower formation node |
| `detector.py` | — | cyan helmet detection, used by `follower` |
| `formations.py` | — | (range, bearing) offset table, used by `follower` |

## Two generations of following

The package contains **two independent following implementations**. They do not
share code and are started by different launch files:

- **`follower_node.py` + `detector.py` + `formations.py`** — the original path.
  One leader wearing a single **cyan** helmet, followers holding a per-formation
  (range, bearing) offset. Started by `follower.launch.py` and by `ttb.sh start`.
- **`tracker_node.py`** — the current path, and the one the GUI and
  `robot_behavior.launch.py mode:=cascade` use. A **colour cascade**: every robot
  wears a different helmet colour and follows the one ahead of it. Colour
  thresholds live in the `COLORS` dict at the top of the file.

If you are changing following behaviour today, `tracker_node.py` is almost
certainly the file you want.

All nodes subscribe with **sensor QoS** (`qos_profile_sensor_data`, BEST_EFFORT).
A RELIABLE subscriber receives nothing from these publishers — that is the first
thing to check when a node sits still and its message counters stay at 0.

---

## `tracker` — colour cascade with lidar relay

Follows the coloured helmet of the robot in front. Default fleet assignment:
tortuga1 yellow (driven by hand), tortuga2 red follows yellow, tortuga3 green
follows red, tortuga4 blue follows green.

**Topics** — subscribes `camera/image_raw` (Image), `scan` (LaserScan), `odom`
(Odometry); publishes `cmd_vel` (Twist) at 10 Hz. All relative, so the node is
namespaced with `-r __ns:=/tortugaX`.

**The lidar relay is the point of the design.** Colour identifies *who* to
follow, but vision drops out constantly in motion. So:

1. colour gives the direction and identifies the right target;
2. the moment a helmet is seen, a lock is armed on that direction;
3. when the colour disappears, the lidar takes over — it finds the nearest
   robot-sized blob in a wide cone around the last known direction and keeps
   following at `target_distance`;
4. only after losing **both** colour and blob for `lidar_lost_time` does the node
   fall back to searching.

A blob is a run of contiguous lidar points at similar range; anything wider than
`blob_max_width` is a wall or furniture, not a robot, and is rejected.

**State machine**, highest priority first:

| State | When |
|---|---|
| `STUCK` | commanded forward but odometry reads ~0 → reverse + turn |
| `AVOID` | obstacle within `obstacle_dist` and not aligned with the target |
| `ARRIVED` | at distance, or the helmet is large in frame (`area_near`) |
| `TRACK` | colour visible → visual following |
| `LIDAR` | colour lost, lock still valid → lidar relay |
| `COAST` / `HOLD` | just lost everything, hold the last heading briefly |
| `SEARCH` | lock dropped → spin in place looking for the colour |

**Key parameters** (full list in the file):

| Parameter | Default | Meaning |
|---|---|---|
| `target_color` | `jaune` | `jaune`/`rouge`/`vert`/`bleu`/`cyan`, English aliases accepted, or `custom` with `hsv_low`/`hsv_high` |
| `target_distance` | 0.32 | distance to hold (m) |
| `desired_bearing` | 0.0 | angle (deg) at which the target should sit; 0 = column, ±25/30 = V formations |
| `safety_dist` | 0.26 | never closer than this on the lidar; reverses below it |
| `obstacle_dist` | 0.45 | avoidance trigger for non-target obstacles |
| `track_cone_deg` | 35.0 | half-cone searched for the relay blob |
| `lidar_lost_time` | 3.0 | seconds without colour *or* blob before searching |
| `blob_max_width` | 0.35 | wider than this is not a robot (m) |

Tunable at runtime:

```bash
ros2 param set /tortuga3/tracker target_distance 0.40
```

The node logs one diagnostic line per second: state, whether the colour is seen,
blob area, target distance, lidar lock angle/distance, nearest obstacle, and the
camera/scan/odom message counters.

> `desired_bearing` is declared as a **double**. Passing an integer (`0` rather
> than `0.0`) makes ROS 2 reject the parameter and the node crashes at startup —
> which is why the GUI wraps every bearing in `float()`.

---

## `wander` — lidar-only wandering

Drives forward, meanders, and avoids everything using the lidar alone. The camera
is never touched, which is what makes it the base of the dataset mode.

**Topics** — subscribes `scan`, `odom`; publishes `cmd_vel`.

**Body-aware avoidance.** The robot is a 16x16 cm square, so its bubble is the
*circumscribed* circle — half-diagonal ≈ 0.113 m plus a margin, hence
`robot_radius` = 0.12 rather than the 0.08 half-width. The corners are what
catch on obstacles while turning, so the node watches a wide ±`avoid_deg` front
cone plus the side sectors, not just the narrow ±`front_deg` cone — a robot
arriving at an angle used to be invisible to the front reading and got clipped on
a corner.

Avoidance runs in two stages: a full stop for `pause_time` (in case the obstacle
moves away on its own), then pure rotation in a locked direction until the wide
cone *and* the bubble are genuinely clear.

The meander is a random angular velocity that *lasts* (renewed every
`wander_min_s`..`wander_max_s`), rather than a jerk every few seconds — the robot
snakes gently instead of walking straight then turning sharply.

| Parameter | Default | Meaning |
|---|---|---|
| `v_forward` | 0.12 | cruise speed (m/s) |
| `obstacle_dist` | 0.50 | stop threshold ahead (m) |
| `safety_dist` | 0.26 | reverse below this (m) |
| `robot_radius` | 0.12 | avoidance bubble, circumscribed circle (m) |
| `front_deg` / `avoid_deg` | 25 / 45 | narrow and wide front half-cones (deg) |
| `wander_w_max` | 0.35 | meander amplitude (rad/s) |
| `wander_min_s` / `wander_max_s` | 3.5 / 7.0 | how long a heading is held (s) |

States: `FORWARD`, `PAUSE`, `REROUTE`, plus an anti-stall reverse when the
odometry says the robot is not moving despite a forward command (a table leg the
lidar cannot see).

---

## `recorder` — dataset capture

Writes a synchronised, directly usable dataset locally on the robot. Already
documented in English in the file's own docstring; in short:

**Topics** — subscribes `camera/image_raw/compressed` (CompressedImage), `scan`,
`odom`.

**Output**, one folder per `segment_minutes` segment:

```
~/dataset/<robot>_<session>_segNN/
    frame_000001.jpg ...   native JPEG bytes, no re-encode (55-60 fps)
    frames.csv             frame, filename, wall_time, ros_sec, ros_nsec, dt_s
    scan.csv               wall_time, ros_sec, ros_nsec, angle_min,
                           angle_increment, range_min, range_max, ranges
    odom.csv               wall_time, ros_sec, ros_nsec, x, y, yaw
```

Every row carries both a millisecond wall clock and the ROS stamp, from the same
clock across all three sensors, so a frame can be matched to a scan and a pose
exactly. Writing native JPEG bytes rather than re-encoding is what lets a Pi
sustain 55 fps where video encoding capped out near 30.

**Disk floor**: below `min_free_mb` (default 700) recording pauses and says so;
it resumes once free space is back above twice the floor. A Pi can never fill its
card. Gaps in the camera stream are detected and logged with their size and frame
index, and SIGTERM closes the segment files cleanly — which is why
`dataset_tools.sh stop` sends SIGTERM to the recorder before killing anything
else.

| Parameter | Default |
|---|---|
| `robot_name` | `tortuga` |
| `segment_minutes` | 5.0 |
| `min_free_mb` | 700.0 |
| `gap_warn_s` | 0.5 |
| `out_dir` | `~/dataset` |

---

## `teleop_zqsd` — keyboard teleop

```bash
ros2 run formation_control teleop_zqsd tortuga1
ros2 run formation_control teleop_zqsd /tortuga1/cmd_vel   # explicit topic
```

`Z`/`S` forward/back, `Q`/`D` turn, `A`/`E` forward while turning, space to stop,
`+`/`-` for speed, `X` to quit (stops the robot on the way out).

Two useful behaviours: given a robot *name*, it scans the ROS graph and publishes
to **every** `cmd_vel` topic belonging to that robot — so it works whether the
namespace ended up single or doubled — and it republishes the last command at
10 Hz, which keeps motion smooth and stops the robot as soon as the process ends.

---

## `follower` — original formation node (legacy path)

Holds a per-formation (range, bearing) offset relative to a leader wearing a
single cyan helmet.

**Topics** — subscribes `camera/image_raw`, `scan`; publishes `cmd_vel`. Unlike
the other nodes it runs its control law inside the image callback, so its rate
follows the camera.

Bearing comes from the camera via `detector.detect_helmet`, range from the lidar
in that direction via `detector.range_from_lidar`, with a crude area-based
fallback when the lidar returns nothing. Proportional control on both errors,
with a deadband on each to stop the robot hunting.

`formation` and `robot_index` are re-read at runtime through a parameter
callback, so `ttb.sh formation` switches formation without restarting anything.

### `detector.py`

- `detect_helmet(bgr_image, min_area_frac)` → `(bearing_norm, area_ratio,
  rectangular)` or `None`. HSV threshold between `CYAN_LOW`/`CYAN_HIGH`,
  open/close morphology, largest contour, centroid → normalised bearing in
  [-1, 1].
- `bearing_to_angle(bearing_norm)` → real angle in the robot frame, positive to
  the left (REP-103), using `CAMERA_HFOV` ≈ 62° (Pi Camera v2).
- `range_from_lidar(scan, angle, window, max_valid)` → nearest valid range in a
  small circular window around that direction, handling both [0, 2π] and [-π, π]
  lidars.

`CYAN_LOW`/`CYAN_HIGH` are the thresholds to recalibrate with
`tools/calibrate_hsv.py` for this path. The cascade path calibrates the `COLORS`
dict in `tracker_node.py` instead, with `tools/hsv_tuner.py`.

### `formations.py`

`FORMATIONS[formation][robot_index] = (range_m, bearing_deg)`, and
`get_offset(formation, robot_index)` returning `(range_m, bearing_rad)` with a
fallback to `colonne`. The table is reproduced in
[`../config/README.md`](../config/README.md).
