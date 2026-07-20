#!/bin/bash
# assemble_video.sh — build a viewable video from a JPEG-sequence segment folder.
#
# The recorder stores native JPEG frames (no re-encode) at up to 55-60 fps.
# This rebuilds an mp4 using the REAL per-frame timestamps from frames.csv, so
# the video timing matches reality (no artificial time jumps). Falls back to a
# fixed frame rate if frames.csv is missing.
#
#   ./assemble_video.sh <segment_folder> [fallback_fps] [out.mp4]
#
# Whole session:
#   for d in ./dataset_collected/tortuga2/*_seg*/; do ./assemble_video.sh "$d"; done

DIR=${1%/}
FPS=${2:-58}
OUT=${3:-${DIR}.mp4}

if [ -z "$DIR" ] || [ ! -d "$DIR" ]; then
  echo "Usage: $0 <segment_folder> [fallback_fps] [out.mp4]"
  exit 1
fi
# Detection insensible au NOMBRE de fichiers : un glob 'ls frame_*.jpg' explose
# en "Argument list too long" sur les gros segments (dizaines de milliers
# d'images) et faisait croire a tort qu'il n'y avait pas de frames. 'find
# -print -quit' s'arrete a la 1re image -> rapide et sans limite d'arguments.
if [ -z "$(find "$DIR" -maxdepth 1 -name 'frame_*.jpg' -print -quit 2>/dev/null)" ]; then
  echo "No frame_*.jpg in $DIR — skipping."
  exit 1
fi

ABS=$(cd "$DIR" && pwd)
CSV="$DIR/frames.csv"

if [ -f "$CSV" ]; then
  # Build an ffmpeg concat list: each frame lasts until the next one, using the
  # REAL ROS timestamps. Capture gaps are REPRESENTED (the frame is held during
  # the gap) instead of being skipped, so the video duration == the real elapsed
  # time -> it stays in sync with the lidar/odom bag (same ROS clock). Only
  # zero/negative deltas fall back to 1/FPS; a huge cap (1h) just guards against
  # a pathological stamp.
  LIST=$(mktemp)
  awk -F, -v DIR="$ABS" -v FPS="$FPS" '
    NR>1 { n++; fn[n]=$2; ts[n]=$3 + $4/1e9 }
    END {
      for (k=1;k<n;k++){
        d = ts[k+1]-ts[k]
        if (d<=0) d = 1.0/FPS
        if (d>3600) d = 3600
        printf "file %s/%s\nduration %.6f\n", DIR, fn[k], d
      }
      if (n>0){
        printf "file %s/%s\nduration %.6f\n", DIR, fn[n], 1.0/FPS
        printf "file %s/%s\n", DIR, fn[n]
      }
    }' "$CSV" > "$LIST"
  echo "Assembling $DIR -> $OUT (real timing from frames.csv)…"
  # -vsync passthrough : garde EXACTEMENT les timestamps du CSV (aucune image
  # jetee/dupliquee, pas de retombee sur 25 fps par defaut). timescale fin pour
  # que les PTS irreguliers (55-60 fps reels) ne soient pas arrondis.
  ffmpeg -y -f concat -safe 0 -i "$LIST" -vsync passthrough \
    -video_track_timescale 90000 -pix_fmt yuv420p "$OUT"
  rm -f "$LIST"
else
  echo "Assembling $DIR -> $OUT (fixed ${FPS} fps, no csv)…"
  ffmpeg -y -framerate "$FPS" -pattern_type glob -i "$DIR/frame_*.jpg" \
    -c:v libx264 -pix_fmt yuv420p "$OUT"
fi
echo "Wrote $OUT"
