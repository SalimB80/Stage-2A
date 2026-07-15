#!/bin/bash
# assemble_video.sh — build a viewable video from a JPEG-sequence segment folder.
#
# The recorder stores native JPEG frames (no re-encode) at up to 55-60 fps.
# This rebuilds an mp4 at a chosen fps for viewing. The per-frame timestamps in
# frames.csv remain the ground truth for training (real timing).
#
#   ./assemble_video.sh <segment_folder> [fps] [out.mp4]
#   ./assemble_video.sh ./dataset_collected/tortuga2/tortuga2_..._seg01 55
#
# To assemble a whole session at once:
#   for d in ./dataset_collected/tortuga2/*_seg*/; do ./assemble_video.sh "$d" 55; done

DIR=${1%/}
FPS=${2:-55}
OUT=${3:-${DIR}.mp4}

if [ -z "$DIR" ] || [ ! -d "$DIR" ]; then
  echo "Usage: $0 <segment_folder> [fps] [out.mp4]"
  exit 1
fi
if ! ls "$DIR"/frame_*.jpg >/dev/null 2>&1; then
  echo "No frame_*.jpg found in $DIR"
  exit 1
fi

echo "Assembling $DIR -> $OUT at ${FPS} fps…"
ffmpeg -y -framerate "$FPS" -pattern_type glob -i "$DIR/frame_*.jpg" \
  -c:v libx264 -pix_fmt yuv420p "$OUT"
echo "Wrote $OUT"
