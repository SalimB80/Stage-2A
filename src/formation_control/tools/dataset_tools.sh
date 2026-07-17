#!/bin/bash
# dataset_tools.sh — pilote la session de collecte dataset depuis le PC.
#
#   ./dataset_tools.sh start 1 2 3 4   -> lance errance+enregistrement
#   ./dataset_tools.sh stop            -> arret PROPRE (recorder d'abord)
#   ./dataset_tools.sh collect         -> rapatrie les videos + segments colles
#                                         (_final.mp4) + rosbags convertis en CSV
#   ./dataset_tools.sh drain 2 3       -> pull + purge EN CONTINU des segments
#                                         termines (disque des Pi toujours bas)
#   ./dataset_tools.sh concat 1 2      -> (re)colle les segments d'une session en
#                                         <robot>_<session>_final.mp4
#   ./dataset_tools.sh bag2csv 1 2     -> convertit les rosbags .db3 deja rapatries
#                                         en CSV (a cote des videos)
#   ./dataset_tools.sh tidy 1 2        -> range un dossier deja rapatrie en
#                                         sous-dossiers par session tortugaX_<session>/
#   ./dataset_tools.sh space           -> espace disque restant par robot

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
ASM_FPS=${ASM_FPS:-58}
assemble_dir() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  [ -f "$d.mp4" ] && return 0
  command -v ffmpeg >/dev/null 2>&1 || { echo "  (ffmpeg absent -> pas de video)"; return 0; }
  bash "$ASM" "$d" "$ASM_FPS" "$d.mp4" >/dev/null 2>&1 && echo "  video: $d.mp4"
}

# Colle les segments mp4 de CHAQUE session d'un dossier robot en un seul
# <robot>_<session>_final.mp4 (ordre seg01..segNN).
finalize_videos() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  ls "$d"/*_seg*.mp4 >/dev/null 2>&1 || return 0
  command -v ffmpeg >/dev/null 2>&1 || { echo "  (ffmpeg absent -> pas de _final.mp4)"; return 0; }
  echo "  concat -> videos finales :"
  bash "$CONCAT" "$d"
}

# Convertit chaque rosbag (.db3 isole OU dossier bag_*/) d'un dossier robot en
# CSV, POSES A COTE des videos (meme dossier). Un CSV par topic (scan/odom/imu).
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

# Range un dossier robot "a plat" en sous-dossiers PAR SESSION
# (tortugaX_<session>/ contenant sa video _final.mp4, ses segments et les CSV
# du rosbag). Rattache chaque rosbag a la session video la plus proche.
tidy_dataset() {
  local d="${1%/}"
  [ -d "$d" ] || return 0
  command -v python3 >/dev/null 2>&1 || { echo "  (python3 absent -> pas de rangement)"; return 0; }
  python3 "$TIDY" "$d"
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
      # SIGTERM au recorder ET au rosbag d'abord (fermeture propre des
      # fichiers : un rosbag tue en -9 devient illisible sans 'ros2 bag
      # reindex'), puis kill global.
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
      # colle les segments d'une meme session en <robot>_<session>_final.mp4
      finalize_videos "./dataset_collected/tortuga$i"
      # range tout par session : tortugaX_<session>/ (video + *_total.csv + raw/)
      tidy_dataset "./dataset_collected/tortuga$i"
    done
    echo "Range par session dans ./dataset_collected/tortugaX/tortugaX_<session>/"
    echo "  (video _final.mp4 + frames/odom/scan_total.csv ; brut dans raw/)"
    ;;
  drain)
    # Vide le disque des robots EN CONTINU pendant l'enregistrement.
    # Le recorder ecrit des DOSSIERS de segment (*_segNN/ pleins de .jpg). Le
    # dossier en cours est modifie en permanence (nouvelles images), les
    # segments TERMINES ne le sont plus. On selectionne les dossiers non
    # modifies depuis >1 min (-mmin +1) -> jamais le segment actif -> on
    # rapatrie puis on SUPPRIME du robot (seulement apres un rsync reussi).
    # Ainsi le disque des Pi reste a ~1-2 segments : capture 55 fps sans limite.
    # Intervalle reglable : DRAIN_INTERVAL=90 ./dataset_tools.sh drain 2 3
    INTERVAL=${DRAIN_INTERVAL:-120}
    mkdir -p ./dataset_collected
    echo "Drain actif sur : ${IDX[*]} (intervalle ${INTERVAL}s). Ctrl-C pour arreter."
    while true; do
      for i in "${IDX[@]}"; do
        mkdir -p ./dataset_collected/tortuga$i
        # dossiers de segment termines (non modifies depuis >1 min)
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
    # (re)colle les segments deja rapatries en <robot>_<session>_final.mp4
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      finalize_videos "./dataset_collected/tortuga$i"
    done
    ;;
  bag2csv)
    # convertit les rosbags .db3 deja rapatries en CSV (a cote des videos)
    for i in "${IDX[@]}"; do
      echo "=== tortuga$i ==="
      convert_bags "./dataset_collected/tortuga$i"
    done
    ;;
  tidy)
    # range un dossier deja rapatrie en sous-dossiers par session
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
    echo "Usage: $0 {start|stop|collect|drain|concat|bag2csv|tidy|space} [index robots...]"
    ;;
esac
