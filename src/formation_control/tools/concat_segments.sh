#!/bin/bash
# concat_segments.sh — colle bout a bout les mp4 d'UNE session en un seul final.
#
# assemble_video.sh produit un mp4 PAR segment :
#   tortuga1_20260716_175709_seg01.mp4 ... _seg05.mp4
# Ce script les concatene dans l'ordre en :
#   tortuga1_20260716_175709_final.mp4
#
#   ./concat_segments.sh <dossier> [prefixe_session]
#
# - <dossier>  : dossier contenant les *_segNN.mp4 (ex: ./dataset_collected/tortuga1)
# - prefixe    : optionnel. Sans lui, TOUTES les sessions du dossier sont
#                traitees (une video finale par prefixe <robot>_<session>).
#
# Concatenation sans re-encodage (-c copy) quand c'est possible ; repli sur un
# re-encodage si les segments ne sont pas copiables tels quels.

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
  # segments tries (seg01, seg02, ...) ; le tri lexical suffit (indices 2 chiffres)
  local segs=()
  while IFS= read -r f; do segs+=("$f"); done \
    < <(ls -1 "$ABS/${prefix}"_seg*.mp4 2>/dev/null | sort)
  if [ ${#segs[@]} -eq 0 ]; then
    echo "  (aucun segment mp4 pour $prefix)"; return 0
  fi
  if [ ${#segs[@]} -eq 1 ]; then
    # une seule partie : on copie simplement en _final
    cp -f "${segs[0]}" "$out"
    echo "  $prefix : 1 segment -> $(basename "$out")"; return 0
  fi

  local list; list=$(mktemp)
  for s in "${segs[@]}"; do
    # echappe les quotes pour le demuxer concat
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
  # decouvre tous les prefixes <robot>_<session> a partir des *_segNN.mp4
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
