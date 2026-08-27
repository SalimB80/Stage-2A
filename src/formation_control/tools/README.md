# tools/ — PC-side tooling

Everything here runs on the **control PC**, not on the robots (the one exception
is `camera_fps.py`, which is deployed with the package and executed on the robot
over SSH). None of it is part of the ROS 2 package's build — these are standalone
scripts.

| Script | What it does |
|---|---|
| `launcher_gui.py` | Tkinter fleet console: bringup, modes, and live inspection tabs |
| `deploy_build.sh` | rsync the package to each robot, then `colcon build` remotely |
| `dataset_tools.sh` | drive a whole dataset session: start / stop / collect / drain / … |
| `assemble_video.sh` | one segment folder of JPEGs → an mp4 with real timing |
| `concat_segments.sh` | the `*_segNN.mp4` of one session → `<robot>_<session>_final.mp4` |
| `rebuild_videos.sh` | rebuild missing `_final.mp4` files from `raw/` |
| `tidy_dataset.py` | flatten a pulled dataset into one self-contained folder per session |
| `bag_to_csv.py` | a `.db3` rosbag → one CSV per topic, with or without ROS installed |
| `camera_fps.py` | library + CLI to set the camera fps through `FrameDurationLimits` |
| `hsv_tuner.py` | live HSV calibration on a robot camera, over the network |
| `calibrate_hsv.py` | the same idea on a local webcam |
| `lidar_monitor.py` | terminal lidar monitor: 8 sectors + obstacle alert |

Requirements: `sshpass`, `rsync`, `ffmpeg` for the video chain; `python3-tk`,
`opencv-python` and a sourced ROS 2 for the GUI and the calibration tools.

---

## Fleet control

### `launcher_gui.py`

The main console. Two layers, matching the launch design: start the **bringup**
(motors + lidar + camera), then a **mode** on top of it — `wander`, `cascade` or
`dataset`.

```bash
python3 launcher_gui.py
```

Left panel, in order: **ROBOTS** (selection + battery), **1 · BUILD**
(*Build + Deploy*), **2 · HARDWARE** (*Start robot*, *Lock FPS*), **3 · MODE**
(*Wander* / *Cascade* / *Dataset*, with the formation dropdown and *Apply live*).
Per-robot **ZQSD** and **Halt** buttons, plus a **STOP — kill everything**.

Inspection tabs: **Log**, **Camera** (raw / colour detection / mask),
**Lidar** (points / sectors / accumulated map), **Topics**, **Formation**
(per-robot colour and distance, *HSV Tuner…*, *Save chain*) and **Dataset**
(*Pull data*, *Delete*).

Worth knowing:

- It does not use the behaviour launch files. It runs the nodes directly over
  SSH with `--ros-args -r __ns:=/tortugaX`, detached under `nohup`, logging to
  `~/tracker_tortugaX.log` on the robot. Without the detach, an SSH hiccup
  SIGHUPs the node and the robot freezes with no trace.
- It re-locks the camera fps ~16 s after bringup, because an fps set at launch
  does not stick (see `camera_fps.py` below).
- The camera stream is heavy, so only **one** robot is subscribed at a time —
  the displayed one, and only while the tab is open.
- Fleet configuration (IPs, helmet colours, per-robot follow distances) is in the
  constants at the top of the file.

### `deploy_build.sh`

```bash
./deploy_build.sh 2 3      # tortuga2 and tortuga3
./deploy_build.sh          # try 1 2 3 4
```

rsyncs the package to each robot and runs `colcon build --symlink-install`
remotely. An unreachable robot is skipped cleanly rather than aborting the run,
and a per-robot summary at the end shows which ones actually got the new code.
Exits non-zero only if *no* robot succeeded.

Prefer this over `../scripts/ttb.sh deploy`, which copies but does not build.

### `lidar_monitor.py`

```bash
python3 lidar_monitor.py tortuga3
```

Answers one question: does the lidar see obstacles? Prints a live bar chart of
the nearest distance in 8 sectors around the robot, with an alert below 0.40 m.
Handles both [0, 2π] and [-π, π] lidars.

---

## Colour calibration

Room lighting shifts the hue, so the thresholds need recalibrating on site.

### `hsv_tuner.py` — on a robot camera, over the network

```bash
python3 hsv_tuner.py tortuga3
```

Keys: `P` prints the current thresholds, `1`/`2`/`3` load yellow/cyan/red
presets, `ESC` quits.

It subscribes to the **compressed** stream first, with sensor QoS. The raw stream
is ~14 MB/s and does not survive Wi-Fi — you get a black screen. It falls back to
raw only if no compressed topic exists.

Copy the values you land on into the `COLORS` dict at the top of
`formation_control/tracker_node.py` — one place to edit — then redeploy.

### `calibrate_hsv.py` — on a local webcam

```bash
python3 calibrate_hsv.py
```

Six trackbars, original and masked image side by side, `ESC` to quit. For the
legacy cyan-helmet path: the values go into `CYAN_LOW`/`CYAN_HIGH` in
`formation_control/detector.py`.

---

## Camera frame rate

### `camera_fps.py`

Both a library and a CLI. libcamera exposes no "fps" setting — the frame rate is
the frame *duration*, so setting fps means writing `FrameDurationLimits =
[1e6/fps, 1e6/fps]` on the `camera_ros` node. Locking min == max forces a fixed
rate.

```bash
python3 camera_fps.py --node /tortuga2/camera --fps 55
python3 camera_fps.py --node /tortuga2/camera --get
python3 camera_fps.py --node /tortuga2/camera --range
```

`--no-lock` caps the rate instead of pinning it, `--force` skips the sensor-mode
range check.

As a library:

```python
from camera_fps import CameraFPSController

with CameraFPSController(camera_node="/camera/camera") as cam:
    cam.set_fps(55.0)
    print(cam.get_fps())
```

The catch, documented at length in the module docstring: exposure time can never
exceed frame duration. If auto-exposure wants 40 ms and you lock to 55 fps
(18 ms), the request is physically impossible — depending on the pipeline,
libcamera shortens the exposure (fine, gain compensates), lets the duration
stretch so your real rate drops, or rejects the control. Use `lock=False` when
correct exposure matters more than a steady rate.

This is also why the GUI re-locks the fps *at runtime*, on the robot, after the
camera has booted: a value set at launch gets overridden by auto-exposure and the
rate collapses to ~18 fps.

> The docstring references a `HOW_IT_WORKS.md` for the full explanation. That
> file is not in this repository.

---

## The dataset pipeline

### End to end

```bash
./dataset_tools.sh start 1 2 3 4     # 1. wander + record on each robot
./dataset_tools.sh drain 2 3         # 2. optional: pull + purge while recording
./dataset_tools.sh stop              # 3. clean stop, recorder first
./dataset_tools.sh collect 1 2 3 4   # 4. pull + assemble + concat + tidy
```

Step 4 does everything on its own: it pulls `~/dataset/` from each robot into
`./dataset_collected/tortugaX/`, assembles each segment into an mp4, joins the
segments of a session into `<robot>_<session>_final.mp4`, converts any rosbags to
CSV, tidies each session into its own folder, and rebuilds any video that went
missing along the way.

Result, per session:

```
dataset_collected/tortuga2/
  tortuga2_20260716_190132/                  <- one session, delete in one go
      tortuga2_20260716_190132_final.mp4     full video
      frames_total.csv                       every segment's frames.csv, merged
      odom_total.csv                         every segment's odometry
      scan_total.csv                         every segment's lidar
      raw/
          tortuga2_20260716_190132_seg01/    images + frames/odom/scan.csv
          tortuga2_20260716_190132_seg01.mp4
          ...
```

The `*_total.csv` files carry a leading `segment` column, so any row can be
traced back to its raw segment.

### `dataset_tools.sh`

```bash
./dataset_tools.sh start 1 2 3 4   # wander + recording
./dataset_tools.sh stop            # CLEAN stop (recorder first)
./dataset_tools.sh collect         # pull + videos + joined segments + CSVs
./dataset_tools.sh drain 2 3       # continuously pull and purge finished segments
./dataset_tools.sh concat 1 2      # (re)join segments into _final.mp4
./dataset_tools.sh bag2csv 1 2     # convert already-pulled .db3 rosbags to CSV
./dataset_tools.sh rebuildvid 1    # rebuild missing videos from raw/
./dataset_tools.sh tidy 1 2        # tidy an already-pulled folder per session
./dataset_tools.sh space           # disk space left on each robot
```

With no robot indices, every subcommand defaults to all four.

**`stop` sends SIGTERM to the recorder first**, then kills the rest. Files get
closed properly; a rosbag killed with `-9` becomes unreadable without a
`ros2 bag reindex`.

**`drain` is what makes long sessions possible.** At 55 fps a Pi's card fills
fast. `drain` repeatedly pulls the segment folders that have not been modified
for over a minute — never the one being written — and deletes them from the robot
*after* a successful rsync. The robot stays at one or two segments on disk, so
recording length stops being bounded by storage. Interval is tunable:
`DRAIN_INTERVAL=90 ./dataset_tools.sh drain 2 3`.

### `assemble_video.sh`

```bash
./assemble_video.sh <segment_folder> [fallback_fps] [out.mp4]
```

Builds an mp4 from a folder of JPEG frames, timed from the **real** per-frame
timestamps in `frames.csv` rather than an assumed constant rate. Capture gaps are
represented (the frame is held) instead of skipped, so the video duration equals
real elapsed time and stays in sync with the lidar/odom data on the same ROS
clock. Falls back to a fixed rate when `frames.csv` is missing.

Whole session at once:

```bash
for d in ./dataset_collected/tortuga2/*_seg*/; do ./assemble_video.sh "$d"; done
```

### `concat_segments.sh`

```bash
./concat_segments.sh <folder> [session_prefix]
```

Joins `*_segNN.mp4` into `<robot>_<session>_final.mp4`, in order. Without a
prefix, every session found in the folder is processed. Uses `-c copy` when
possible, falls back to a re-encode otherwise.

### `rebuild_videos.sh`

```bash
./rebuild_videos.sh <folder> [fps] [force]
```

Recovery tool: after tidying, segments live in `<session>/raw/`. If the full
video was never produced, this rebuilds it — assemble each segment folder, then
concatenate. `<folder>` can be `dataset_collected`, one robot, or one session.
`force` also redoes sessions whose `_final.mp4` already exists, which is what you
want if earlier videos have bad timing.

Segment mp4s already sitting in `raw/` are **never reused** — they are rebuilt
from the images, because old encodes at a forced fps would contaminate the final
video.

### `tidy_dataset.py`

```bash
./tidy_dataset.py <folder> [--dry-run] [--tol SECONDS]
```

Turns a flat `dataset_collected/tortugaX/` into the per-session layout shown
above, and builds the `*_total.csv` files. `<folder>` can be one robot folder or
the parent, in which case every `tortugaX/` is processed. `--tol` (default 300 s)
is how close an old rosbag's timestamp must be to a session to be filed with it.

It only ever *moves* files, never deletes, skips existing destinations, and is
idempotent. CSV reading tolerates damage: NUL bytes from a recorder killed
mid-write are stripped, and one corrupt line stops that file without breaking the
run. Start with `--dry-run`.

### `bag_to_csv.py`

```bash
./bag_to_csv.py <bag.db3 | bag_folder> [output_folder]
```

A `.db3` is a SQLite database, unreadable as-is. This deserialises it into one
CSV per topic, written next to the video by default. It accepts a standalone
`.db3` or a full bag folder.

Two decoding paths: if ROS 2 is sourced it handles **any** message type
generically; otherwise it falls back to a built-in CDR decoder covering
LaserScan, Odometry and Imu — so no ROS installation is needed on the PC.

Every row carries `bag_time_ns` and, when the message has a header, `stamp_sec` /
`stamp_nanosec`, directly alignable with the recorder's `frames.csv`, `scan.csv`
and `odom.csv` (same ROS clock).

> Current dataset runs record **no rosbag** — the recorder writes the CSVs
> itself. This tool is for bags captured before that change.
