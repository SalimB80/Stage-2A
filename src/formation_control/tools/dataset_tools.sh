#!/bin/bash
# dataset_tools.sh — drive the dataset collection session from the PC.
#
#   ./dataset_tools.sh start 1 2 3 4   -> start wandering + recording
#   ./dataset_tools.sh stop            -> CLEAN stop (recorder first)
#   ./dataset_tools.sh collect         -> pull the videos + joined segments
#                                         (_final.mp4) + rosbags converted to CSV
#   ./dataset_tools.sh drain 2 3       -> CONTINUOUSLY pull and purge finished
#                                         segments (keeps the Pi disks low)
#   ./dataset_tools.sh concat 1 2      -> (re)join a session's segments into
#                                         <robot>_<session>_final.mp4
#   ./dataset_tools.sh bag2csv 1 2     -> convert the already-pulled .db3 rosbags
#                                         to CSV (next to the videos)
#   ./dataset_tools.sh tidy 1 2        -> tidy an already-pulled folder into
#                                         per-session tortugaX_<session>/ subfolders
#   ./dataset_tools.sh space           -> disk space left on each robot

PW=1234
ENV="export ROS_DOMAIN_ID=30; export TURTLEBOT3_MODEL=burger; \
export LDS_MODEL=LDS-03; source /opt/ros/humble/setup.bash; \
source ~/turtlebot3_ws/install/setup.bash; \
source ~/formation_ws/install/setup.bash;"

ALL=(1 2 3 4)
CMD=$1; shift
IDX=("$@"); [ ${#IDX[@]} -eq 0 ] && IDX=("${ALL[@]}")

run_ssh() { sshpass -p $PW ssh -o StrictHostKeyChecking=no \
            -o ConnectTimeout=4 tortuga$1@192.168.0.20$1 "$2"; }

# Auto-assemble a segment folder into <folder>.mp4 (skips if already done or if
# ffmpeg/assemble_video.sh is missing). Timing comes from frames.csv.
ASM="$(dirname "$0")/assemble_video.sh"
CONCAT="$(dirname "$0")/concat_segments.sh"
BAG2CSV="$(dirname "$0")/bag_to_csv.py"
TIDY="$(dirname "$0")/tidy_dataset.py"
REBUILD="$(dirname "$0")/rebuild_videos.sh"
ASM_FPS=${ASM_FPS:-58}
assemble_dir() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  [ -f "$d.mp4" ] && return 0
  command -v ffmpeg >/dev/null 2>&1 || { echo "  (ffmpeg absent -> pas de video)"; return 0; }
  bash "$ASM" "$d" "$ASM_FPS" "$d.mp4" >/dev/null 2>&1 && echo "  video: $d.mp4"
}

# Join the mp4 segments of EVERY session in a robot folder into a single
# <robot>_<session>_final.mp4 (in seg01..segNN order).
finalize_videos() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  ls "$d"/*_seg*.mp4 >/dev/null 2>&1 || return 0
  command -v ffmpeg >/dev/null 2>&1 || { echo "  (ffmpeg absent -> pas de _final.mp4)"; return 0; }
  echo "  concat -> videos finales :"
  bash "$CONCAT" "$d"
}

# Convert every rosbag (standalone .db3 OR bag_*/ folder) of a robot folder to
# CSV, placed NEXT TO the videos (same folder). One CSV per topic (scan/odom/imu).
convert_bags() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  command -v python3 >/dev/null 2>&1 || { echo "  (python3 absent -> pas de conversion db3)"; return 0; }
  local b
  for b in "$d"/bag_*/ "$d"/*.db3; do
    [ -e "$b" ] || continue
    python3 "$BAG2CSV" "$b" "$d"
  done
}

# Tidy a "flat" robot folder into PER-SESSION subfolders (tortugaX_<session>/
# holding its _final.mp4 video, its segments and the rosbag CSVs). Attaches each
# rosbag to the closest video session.
tidy_dataset() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  command -v python3 >/dev/null 2>&1 || { echo "  (python3 absent -> pas de rangement)"; return 0; }
  python3 "$TIDY" "$d"
}

# Safety net: after tidying, rebuild any missing _final.mp4 video FROM raw/
# (assemble the segments + concatenate). Skips the ones already OK and reports
# the sessions with no frames.
rebuild_videos() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  command -v ffmpeg >/dev/null 2>&1 || return 0
  bash "$REBUILD" "$d" "$ASM_FPS"
}

case $CMD in
  start)
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i : lancement dataset ==="
      run_ssh $i "$ENV nohup ros2 launch formation_control \
        robot_dataset.launch.py namespace:=tortuga$i \
        > ~/dataset_launch.log 2>&1 &" &
    done
    wait
    echo "Session lancee. Les robots errent et enregistrent en local."
    ;;
  stop)
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i : arret propre ==="
      # SIGTERM to the recorder AND the rosbag first (so the files are closed
      # cleanly: a rosbag killed with -9 becomes unreadable without a
      # 'ros2 bag reindex'), then a global kill.
      run_ssh $i "pkill -TERM -f '[r]ecorder'; \
                  pkill -TERM -f '[b]ag record'; sleep 2; \
                  pkill -9 -f '[r]os2 launch'; \
                  pkill -9 -f -- '[-]-ros-args'; true" &
    done
    wait
    echo "Arret termine."
    ;;
  collect)
    mkdir -p ./dataset_collected
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i : rapatriement ==="
      mkdir -p ./dataset_collected/tortuga$i
      sshpass -p $PW rsync -avz --progress \
        -e "ssh -o StrictHostKeyChecking=no" \
        tortuga$i@192.168.0.20$i:~/dataset/ ./dataset_collected/tortuga$i/
      # auto-assemble each pulled segment into a ready-to-watch mp4
      for d in ./dataset_collected/tortuga$i/*_seg*/; do
        assemble_dir "$d"
      done
      # join the segments of one session into <robot>_<session>_final.mp4
      finalize_videos "./dataset_collected/tortuga$i"
      # tidy everything per session: tortugaX_<session>/ (video + *_total.csv + raw/)
      tidy_dataset "./dataset_collected/tortuga$i"
      # safety net: rebuild any missing video from raw/
      rebuild_videos "./dataset_collected/tortuga$i"
    done
    echo "Range par session dans ./dataset_collected/tortugaX/tortugaX_<session>/"
    echo "  (video _final.mp4 + frames/odom/scan_total.csv ; brut dans raw/)"
    ;;
  drain)
    # Empty the robots' disks CONTINUOUSLY while recording.
    # The recorder writes segment FOLDERS (*_segNN/ full of .jpg). The current
    # folder is modified constantly (new images), FINISHED segments are not.
    # We select the folders untouched for >1 min (-mmin +1) -> never the active
    # segment -> pull them, then DELETE them from the robot (only after a
    # successful rsync). The Pi disks therefore stay at ~1-2 segments: 55 fps
    # capture with no limit.
    # Tunable interval: DRAIN_INTERVAL=90 ./dataset_tools.sh drain 2 3
    INTERVAL=${DRAIN_INTERVAL:-120}
    mkdir -p ./dataset_collected
    echo "Drain actif sur : ${IDX[*]} (intervalle ${INTERVAL}s). Ctrl-C pour arreter."
    while true; do
      for i in "${IDX[@]}"; do
        mkdir -p ./dataset_collected/tortuga$i
        # finished segment folders (untouched for >1 min)
        DIRS=$(run_ssh $i "find ~/dataset -maxdepth 1 -type d -name '*_seg*' \
                -mmin +1 -printf '%f\n' 2>/dev/null")
        [ -z "$DIRS" ] && continue
        n=$(echo "$DIRS" | grep -c . )
        echo "=== tortuga$i : $n segment(s) termine(s) a rapatrier ==="
        while IFS= read -r d; do
          [ -z "$d" ] && continue
          if sshpass -p $PW rsync -az \
               -e "ssh -o StrictHostKeyChecking=no" \
               tortuga$i@192.168.0.20$i:~/dataset/"$d" \
               ./dataset_collected/tortuga$i/ ; then
            run_ssh $i "rm -rf ~/dataset/'$d'"  # purge SEULEMENT si rsync OK
            echo "  ok + purge : $d"
            assemble_dir "./dataset_collected/tortuga$i/$d"   # -> mp4 auto
          else
            echo "  echec transfert (conserve sur le robot) : $d"
          fi
        done <<< "$DIRS"
      done
      sleep "$INTERVAL"
    done
    ;;
  concat)
    # (re)join the already-pulled segments into <robot>_<session>_final.mp4
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      finalize_videos "./dataset_collected/tortuga$i"
    done
    ;;
  bag2csv)
    # convert the already-pulled .db3 rosbags to CSV (next to the videos)
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      convert_bags "./dataset_collected/tortuga$i"
    done
    ;;
  rebuildvid)
    # rebuild the missing _final.mp4 videos from raw/ (already tidied)
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      rebuild_videos "./dataset_collected/tortuga$i"
    done
    ;;
  tidy)
    # tidy an already-pulled folder into per-session subfolders
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      tidy_dataset "./dataset_collected/tortuga$i"
    done
    ;;
  space)
    for i in "${IDX[@]}"; do
      echo -n "tortuga$i : "
      run_ssh $i "df -h ~ | tail -1 | awk '{print \$4\" libres\"}'; \
                  du -sh ~/dataset 2>/dev/null || echo '0 dataset'"
    done
    ;;
  *)
    echo "Usage: $0 {start|stop|collect|drain|concat|bag2csv|tidy|rebuildvid|space} [index robots...]"
    ;;
esac
