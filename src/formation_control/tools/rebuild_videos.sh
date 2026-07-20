#!/bin/bash
# rebuild_videos.sh — (re)construit les <session>_final.mp4 MANQUANTS depuis raw/.
#
# Apres rangement, les segments sont dans <session>/raw/. Si la video complete
# n'a jamais ete generee (assemblage rate, tidy lance seul, etc.), ce script la
# reconstruit : il assemble chaque dossier de segment de raw/ puis les concatene
# en <session>/<session>_final.mp4. Saute les sessions deja OK, signale celles
# sans frames (camera muette).
#
#   ./rebuild_videos.sh <dossier> [fps] [force]
#
# <dossier> = dataset_collected (tous robots) OU un robot OU une seule session.
# force     = refait AUSSI les sessions dont le _final.mp4 existe deja (utile si
#             les videos precedentes ont un mauvais timing).
#
# IMPORTANT : les segNN.mp4 deja presents dans raw/ ne sont JAMAIS reutilises —
# ils sont reconstruits depuis les images (frame_*.jpg + frames.csv). D'anciens
# segments encodes avec un fps force contamineraient sinon la video finale.

ROOT=${1%/}
FPS=${2:-58}
FORCE=0
case "$2$3" in *force*) FORCE=1;; esac
[ "$FPS" = "force" ] && FPS=58     # './rebuild_videos.sh <dir> force' aussi valide
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
    return 0                                   # deja fait (relancer avec 'force')
  fi

  # 1) REconstruit chaque segment DEPUIS LES IMAGES raw. Les segNN.mp4 deja
  #    presents sont ecartes (supprimes puis refaits) : d'anciens encodages a
  #    fps force fausseraient le timing de la video finale.
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

  # 2) des mp4 de segment ont-ils ete produits ? (sinon = pas de frames)
  if ! ls "$raw/${pref}"_seg*.mp4 >/dev/null 2>&1; then
    echo "  ! $pref : AUCUNE FRAME -> pas de video (camera muette ?)"
    return 0
  fi

  # 3) concatene -> raw/<pref>_final.mp4, puis remonte au sommet de la session
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
