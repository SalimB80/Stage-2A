# config/ — parameters and DDS profile

Two unrelated files: one holds node parameters, the other fixes DDS discovery on
a network where multicast does not work.

| File | Purpose |
|---|---|
| `formations.yaml` | ROS 2 parameters for the `follower` node |
| `fastdds_unicast.xml` | Fast DDS unicast profile, for when multicast is blocked |

Both are installed to `share/formation_control/config/` by `setup.py`, so they
are available on the robots after a `colcon build`.

## `formations.yaml`

Parameters applied to every `follower` node, whatever its namespace (the
`/**/follower` wildcard key):

| Parameter | Value | Meaning |
|---|---|---|
| `max_lin` | 0.18 | linear speed cap (m/s) |
| `max_ang` | 1.0 | angular speed cap (rad/s) |
| `k_lin` | 0.6 | proportional gain on the range error |
| `k_ang` | 1.5 | proportional gain on the bearing error |
| `stop_range` | 0.25 | below this range (m), forward motion is clamped to 0 |
| `search_when_lost` | true | spin in place after the leader has been lost |

Use it with:

```bash
ros2 run formation_control follower --ros-args --params-file \
  $(ros2 pkg prefix formation_control)/share/formation_control/config/formations.yaml
```

Two things worth knowing:

- **It only covers `follower`.** `tracker_node` — the node used by the current
  colour cascade — is not matched by the `/**/follower` key. Its defaults live in
  `formation_control/tracker_node.py` and are overridden on the command line by
  the GUI and by `robot_behavior.launch.py`.
- **The gains here differ from the node's own defaults** (`k_lin` 0.6 vs 0.35,
  `k_ang` 1.5 vs 0.8 in `follower_node.py`). Whichever is loaded last wins, so be
  explicit about whether you are passing this file.

The *geometry* of the formations is not in this file: the per-follower
(range, bearing) offsets are in `formation_control/formations.py`, as a plain
Python dict.

| Formation | tortuga2 | tortuga3 | tortuga4 |
|---|---|---|---|
| `colonne` | 0.6 m, 0° | 1.2 m, 0° | 1.8 m, 0° |
| `ligne` | 0.6 m, +30° | 0.6 m, -30° | 1.0 m, 0° |
| `triangle` | 0.6 m, +25° | 0.6 m, -25° | 1.2 m, 0° |
| `carre` | 0.6 m, +20° | 0.6 m, -20° | 1.0 m, 0° |

A positive bearing means the leader is seen on the left, so the follower places
itself on the right.

## `fastdds_unicast.xml`

ROS 2 discovers nodes over DDS multicast by default. WSL2 behind a NAT, and some
managed Wi-Fi networks, drop multicast — the robots and the PC then never see
each other even though `ping` works fine.

This profile replaces discovery with an explicit list of peers, one per robot
(`192.168.0.201` .. `.204`), and declares itself the default participant profile.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/formation_ws/src/formation_control/config/fastdds_unicast.xml
```

Notes:

- **Export it on every machine**, the PC *and* each robot. Discovery is
  symmetric: a participant that still relies on multicast will not be found.
- It requires `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (already exported by
  `ttb.sh` and `dataset_tools.sh`).
- The peer list is hardcoded to the four fleet IPs. Adding a robot means adding
  a `<locator>` entry here.
- The PC itself is not in the list. Robots reach it through the peers that
  contact them first; if PC-side discovery misbehaves, add the PC's address too.

Try mirrored WSL networking first (see the [root README](../../../README.md)) —
this profile is the fallback when that is not enough.
