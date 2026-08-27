#!/bin/bash
# rebuild_videos.sh — (re)build the MISSING <session>_final.mp4 files from raw/.
#
# After tidying, the segments live in <session>/raw/. If the full video was never
# generated (failed assembly, tidy run on its own, etc.), this script rebuilds
# it: it assembles every segment folder in raw/ then concatenates them into
# <session>/<session>_final.mp4. Sessions already OK are skipped, and those with
# no frames (silent camera) are reported.
#
#   ./rebuild_videos.sh <folder> [fps] [force]
#
# <folder> = dataset_collected (all robots) OR one robot OR a single session.
# force    = ALSO redo the sessions whose _final.mp4 already exists (useful when
#            the previous videos have bad timing).
#
# IMPORTANT: the segNN.mp4 files already present in raw/ are NEVER reused — they
# are rebuilt from the images (frame_*.jpg + frames.csv). Old segments encoded
# with a forced fps would otherwise contaminate the final video.

ROOT=${1%/}
FPS=${2:-58}
FORCE=0
case "$2$3" in *force*) FORCE=1;; esac
[ "$FPS" = "force" ] && FPS=58     # './rebuild_videos.sh <dir> force' is valid too
HERE=$(dirname "$0")
ASM="$HERE/assemble_video.sh"
CONCAT="$HERE/concat_segments.sh"

if [ -z "$ROOT" ] || [ ! -d "$ROOT" ]; then
  echo "Usage: $0 <dossier> [fps]"; exit 1
fi
if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg absent -> impossible de reconstruire les videos."; exit 1
fi

rebuild_session() {
  local S=${1%/}
  local raw="$S/raw"
  local pref; pref=$(basename "$S")
  local final="$S/${pref}_final.mp4"
  [ -d "$raw" ] || return 0
  if [ -s "$final" ] && [ "$FORCE" != 1 ]; then
    return 0                                   # already done (re-run with 'force')
  fi

  # 1) REbuild each segment FROM THE RAW IMAGES. The segNN.mp4 files already
  #    present are discarded (deleted then redone): old encodes at a forced fps
  #    would otherwise skew the timing of the final video.
  local any_seg=0
  for d in "$raw"/*_seg*/; do
    [ -d "$d" ] || continue
    any_seg=1
    local m="${d%/}.mp4"
    rm -f "$m"
    bash "$ASM" "$d" "$FPS" "$m" >/dev/null 2>&1
  done
  if [ "$any_seg" = 0 ]; then
    echo "  ! $pref : aucun dossier de segment dans raw/ (rien a reconstruire)"
    return 0
  fi

  # 2) were any segment mp4s produced? (if not = no frames)
  if ! ls "$raw/${pref}"_seg*.mp4 >/dev/null 2>&1; then
    echo "  ! $pref : AUCUNE FRAME -> pas de video (camera muette ?)"
    return 0
  fi

  # 3) concatenate -> raw/<pref>_final.mp4, then move it up to the session root
  bash "$CONCAT" "$raw" "$pref" >/dev/null 2>&1
  if [ -s "$raw/${pref}_final.mp4" ]; then
    mv -f "$raw/${pref}_final.mp4" "$final"
    echo "  video: $pref/${pref}_final.mp4"
  else
    echo "  ! $pref : concat echoue"
  fi
}

n=0
while IFS= read -r raw; do
  rebuild_session "$(dirname "$raw")"
  n=$((n + 1))
done < <(find "$ROOT" -type d -name raw | sort)

[ "$n" = 0 ] && echo "Aucune session (dossier raw/) trouvee sous $ROOT."
echo "Termine."
