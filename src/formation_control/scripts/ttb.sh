#!/usr/bin/env bash
# ============================================================
#  ttb.sh - Controle unifie de la flotte TurtleBot3
# ============================================================
#  Usage :
#    ./ttb.sh deploy  [N]              copie + build sur N robots
#    ./ttb.sh start   [N] [formation]  lance bringup + followers
#    ./ttb.sh stop    [N]              arrete tous les noeuds
#    ./ttb.sh monitor [N]              grille tmux des logs
#    ./ttb.sh teleop                   pilote le leader au clavier
#    ./ttb.sh formation [N] [form]     change la formation a chaud
#
#  Defauts : N=4, formation=colonne
#  Formations : colonne | ligne | triangle | carre
#  Prerequis WSL : sudo apt install sshpass rsync tmux
# ============================================================
set -e

PASS="1234"
PKG="$HOME/formation_ws/src/formation_control"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
ENV="export ROS_DOMAIN_ID=30; export TURTLEBOT3_MODEL=burger; \
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
source /opt/ros/humble/setup.bash; \
source ~/turtlebot3_ws/install/setup.bash; \
source ~/formation_ws/install/setup.bash;"

user_of() { echo "tortuga$1"; }
ip_of()   { echo "192.168.0.20$1"; }

ssh_run() {  # ssh_run <i> <commande...>
  local i=$1; shift
  sshpass -p "$PASS" ssh $SSH_OPTS "$(user_of $i)@$(ip_of $i)" "$@"
}

ssh_bg() {   # ssh_bg <i> <commande...>  (arriere-plan + log)
  local i=$1; shift
  sshpass -p "$PASS" ssh $SSH_OPTS "$(user_of $i)@$(ip_of $i)" \
    "$ENV nohup bash -c '$*' > ~/ros_$i.log 2>&1 &"
}

cmd_deploy() {
  local N=${1:-4}
  for i in $(seq 1 $N); do
    echo "=== [$(user_of $i) @ $(ip_of $i)] copie ==="
    ssh_run $i "mkdir -p ~/formation_ws/src/formation_control"
    sshpass -p "$PASS" rsync -az --delete -e "ssh $SSH_OPTS" \
      "$PKG/" "$(user_of $i)@$(ip_of $i):~/formation_ws/src/formation_control/"
    echo "=== [$(user_of $i)] OK ==="
  done
  echo "Deploiement termine sur $N robot(s)."
}

cmd_start() {
  local N=${1:-4}
  local FORM=${2:-colonne}
  echo "Leader tortuga1 : bringup"
  ssh_bg 1 "ros2 launch turtlebot3_bringup robot.launch.py namespace:=tortuga1"
  for i in $(seq 2 $N); do
    echo "Follower tortuga$i : bringup + follower ($FORM)"
    ssh_bg $i "ros2 launch turtlebot3_bringup robot.launch.py namespace:=tortuga$i"
    sleep 3
    ssh_bg $i "ROS_NAMESPACE=tortuga$i ros2 run formation_control follower --ros-args \
      -p robot_index:=$i -p formation:=$FORM"
  done
  echo "Tout est lance. Logs : ~/ros_<i>.log sur chaque robot."
  echo "Pilote le leader :  ./ttb.sh teleop"
}

cmd_stop() {
  local N=${1:-4}
  for i in $(seq 1 $N); do
    echo "Arret tortuga$i"
    ssh_run $i "pkill -f ros2; pkill -f robot.launch; pkill -f follower" || true
  done
  echo "Tous les noeuds arretes."
}

cmd_monitor() {
  local N=${1:-4}
  tmux kill-session -t ttb 2>/dev/null || true
  tmux new-session -d -s ttb
  for i in $(seq 1 $N); do
    [ $i -gt 1 ] && tmux split-window -t ttb
    tmux send-keys -t ttb \
      "sshpass -p $PASS ssh $SSH_OPTS $(user_of $i)@$(ip_of $i) 'tail -f ~/ros_$i.log'" C-m
    tmux select-layout -t ttb tiled
  done
  tmux attach -t ttb
}

cmd_teleop() {
  $ENV ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tortuga1/cmd_vel
}

cmd_formation() {
  local N=${1:-4}
  local FORM=${2:-colonne}
  for i in $(seq 2 $N); do
    eval "$ENV ros2 param set /tortuga$i/follower formation $FORM"
  done
  echo "Formation changee -> $FORM"
}

case "$1" in
  deploy)    shift; cmd_deploy "$@" ;;
  start)     shift; cmd_start "$@" ;;
  stop)      shift; cmd_stop "$@" ;;
  monitor)   shift; cmd_monitor "$@" ;;
  teleop)    shift; cmd_teleop "$@" ;;
  formation) shift; cmd_formation "$@" ;;
  *)
    echo "Usage : ./ttb.sh {deploy|start|stop|monitor|teleop|formation} [N] [formation]"
    echo "Ex :    ./ttb.sh deploy 4"
    echo "        ./ttb.sh start 3 triangle"
    echo "        ./ttb.sh teleop"
    echo "        ./ttb.sh stop 3"
    exit 1 ;;
esac
