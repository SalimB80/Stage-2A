#!/bin/bash
set -e

ROBOTS=(1 2 3 4)
PKG=~/formation_ws/src/formation_control
PW=1234

if [ $# -gt 0 ]; then ROBOTS=("$@"); fi

for i in "${ROBOTS[@]}"; do
  user="tortuga$i"
  ip="192.168.0.20$i"
  echo "=== [$user @ $ip] copie du package ==="
  sshpass -p "$PW" rsync -az --delete \
    --exclude='dataset_collected/' \
    --exclude='dataset/' \
    --exclude='__pycache__/' \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='*.avi' \
    --exclude='*.mp4' \
    --exclude='*.bag' \
    --exclude='*.db3' \
    -e "ssh -o StrictHostKeyChecking=no" \
    "$PKG/" "$user@$ip:~/formation_ws/src/formation_control/"

  echo "=== [$user] colcon build ==="
  sshpass -p "$PW" ssh -o StrictHostKeyChecking=no "$user@$ip" \
    "source /opt/ros/humble/setup.bash && \
     cd ~/formation_ws && colcon build --symlink-install"
  echo "=== [$user] OK ==="
done
echo "Build distant termine sur : ${ROBOTS[*]}"
