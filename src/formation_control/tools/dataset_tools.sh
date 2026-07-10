#!/bin/bash
# dataset_tools.sh — pilote la session de collecte dataset depuis le PC.
#
#   ./dataset_tools.sh start 1 2 3 4   -> lance errance+enregistrement
#   ./dataset_tools.sh stop            -> arret PROPRE (recorder d'abord)
#   ./dataset_tools.sh collect         -> rapatrie les videos sur le PC
#   ./dataset_tools.sh drain 2 3       -> pull + purge EN CONTINU des segments
#                                         termines (disque des Pi toujours bas)
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
    done
    echo "Videos dans ./dataset_collected/"
    ;;
  drain)
    # Vide le disque des robots EN CONTINU pendant l'enregistrement.
    # Principe : le recorder ecrit des segments fermes ; le fichier en cours
    # d'ecriture est modifie en permanence (60 fps), les segments TERMINES ne
    # le sont plus. On selectionne donc ce qui n'a pas ete modifie depuis >1
    # min (-mmin +1) -> jamais le segment actif -> on rapatrie puis on SUPPRIME
    # du robot (uniquement apres un rsync reussi). Ainsi le disque des Pi reste
    # a ~1-2 segments et on peut filmer a 60 fps sans limite de duree.
    # Intervalle reglable : DRAIN_INTERVAL=90 ./dataset_tools.sh drain 2 3
    INTERVAL=${DRAIN_INTERVAL:-120}
    mkdir -p ./dataset_collected
    echo "Drain actif sur : ${IDX[*]} (intervalle ${INTERVAL}s). Ctrl-C pour arreter."
    while true; do
      for i in "${IDX[@]}"; do
        mkdir -p ./dataset_collected/tortuga$i
        # segments termines (video + csv) non modifies depuis >1 min
        FILES=$(run_ssh $i "find ~/dataset -maxdepth 1 -type f \
                \( -name '*.avi' -o -name '*.csv' \) -mmin +1 -printf '%f\n' \
                2>/dev/null")
        [ -z "$FILES" ] && continue
        n=$(echo "$FILES" | grep -c . )
        echo "=== tortuga$i : $n fichier(s) termine(s) a rapatrier ==="
        while IFS= read -r f; do
          [ -z "$f" ] && continue
          if sshpass -p $PW rsync -az \
               -e "ssh -o StrictHostKeyChecking=no" \
               tortuga$i@192.168.0.20$i:~/dataset/"$f" \
               ./dataset_collected/tortuga$i/ ; then
            run_ssh $i "rm -f ~/dataset/'$f'"   # purge SEULEMENT si rsync OK
            echo "  ok + purge : $f"
          else
            echo "  echec transfert (conserve sur le robot) : $f"
          fi
        done <<< "$FILES"
      done
      sleep "$INTERVAL"
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
    echo "Usage: $0 {start|stop|collect|drain|space} [index robots...]"
    ;;
esac
