#!/bin/bash
# concat_segments.sh — join the mp4 files of ONE session end to end into one final.
#
# assemble_video.sh produces one mp4 PER segment:
#   tortuga1_20260716_175709_seg01.mp4 ... _seg05.mp4
# This script concatenates them in order into:
#   tortuga1_20260716_175709_final.mp4
#
#   ./concat_segments.sh <folder> [session_prefix]
#
# - <folder>  : folder holding the *_segNN.mp4 (e.g. ./dataset_collected/tortuga1)
# - prefix    : optional. Without it, EVERY session in the folder is processed
#               (one final video per <robot>_<session> prefix).
#
# Concatenation without re-encoding (-c copy) whenever possible; falls back to a
# re-encode when the segments cannot be copied as-is.

DIR=${1%/}
PREFIX=$2

if [ -z "$DIR" ] || [ ! -d "$DIR" ]; then
  echo "Usage: $0 <dossier> [prefixe_session]"
  exit 1
fi
if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg absent -> impossible de concatener."
  exit 1
fi

ABS=$(cd "$DIR" && pwd)

concat_one() {
  local prefix="$1"
  local out="$ABS/${prefix}_final.mp4"
  # sorted segments (seg01, seg02, ...); a lexical sort is enough (2-digit index)
  local segs=()
  while IFS= read -r f; do segs+=("$f"); done \
    < <(ls -1 "$ABS/${prefix}"_seg*.mp4 2>/dev/null | sort)
  if [ ${#segs[@]} -eq 0 ]; then
    echo "  (aucun segment mp4 pour $prefix)"; return 0
  fi
  if [ ${#segs[@]} -eq 1 ]; then
    # a single part: just copy it to _final
    cp -f "${segs[0]}" "$out"
    echo "  $prefix : 1 segment -> $(basename "$out")"; return 0
  fi

  local list; list=$(mktemp)
  for s in "${segs[@]}"; do
    # escape the quotes for the concat demuxer
    printf "file '%s'\n" "${s//\'/\'\\\'\'}" >> "$list"
  done

  echo "  $prefix : ${#segs[@]} segments -> $(basename "$out")"
  if ffmpeg -y -f concat -safe 0 -i "$list" -c copy "$out" >/dev/null 2>&1; then
    :
  else
    echo "    (-c copy impossible, re-encodage…)"
    ffmpeg -y -f concat -safe 0 -i "$list" -vsync vfr -pix_fmt yuv420p \
      "$out" >/dev/null 2>&1 \
      || echo "    ECHEC concat pour $prefix"
  fi
  rm -f "$list"
}

if [ -n "$PREFIX" ]; then
  concat_one "$PREFIX"
else
  # discover every <robot>_<session> prefix from the *_segNN.mp4 files
  mapfile -t prefixes < <(
    ls -1 "$ABS"/*_seg*.mp4 2>/dev/null \
      | sed -E 's#.*/##; s/_seg[0-9]+\.mp4$//' | sort -u)
  if [ ${#prefixes[@]} -eq 0 ]; then
    echo "Aucun *_segNN.mp4 dans $DIR."
    exit 0
  fi
  for p in "${prefixes[@]}"; do
    concat_one "$p"
  done
fi
