#!/bin/bash
# deploy_build.sh — copy the package onto each robot, then build it remotely.
#
# Robust: an unreachable robot is skipped cleanly (it no longer fails the whole
# deployment, unlike the old 'set -e'). Creates the missing directory tree
# before copying. Prints a clear PER-ROBOT SUMMARY at the end -> you can see at
# a glance which ones actually received the new code.
#
#   ./deploy_build.sh 2 3      -> tortuga2 and tortuga3
#   ./deploy_build.sh          -> try 1 2 3 4

ROBOTS=(1 2 3 4)
PKG=~/formation_ws/src/formation_control
PW=1234
[ $# -gt 0 ] && ROBOTS=("$@")

ok=(); ko=()
for i in "${ROBOTS[@]}"; do
  user="tortuga$i"; ip="192.168.0.20$i"
  echo "=== [$user @ $ip] ==="

  if ! sshpass -p "$PW" ssh -o StrictHostKeyChecking=no \
        -o ConnectTimeout=4 "$user@$ip" "true" 2>/dev/null; then
    echo "  injoignable -> ignore"; ko+=("$i"); continue
  fi

  sshpass -p "$PW" ssh -o StrictHostKeyChecking=no "$user@$ip" \
    "mkdir -p ~/formation_ws/src/formation_control"

  if ! sshpass -p "$PW" rsync -az --delete \
        -e "ssh -o StrictHostKeyChecking=no" \
        "$PKG/" "$user@$ip:~/formation_ws/src/formation_control/"; then
    echo "  echec copie -> ignore"; ko+=("$i"); continue
  fi

  # build; the output is captured so a failure can be diagnosed
  if sshpass -p "$PW" ssh -o StrictHostKeyChecking=no "$user@$ip" \
        "source /opt/ros/humble/setup.bash && cd ~/formation_ws && \
         colcon build --symlink-install" 2>&1 | tail -3; then
    echo "  OK"; ok+=("$i")
  else
    echo "  echec build"; ko+=("$i")
  fi
done

echo "-------------------------------------------"
echo "Deploy OK      : ${ok[*]:-aucun}"
[ ${#ko[@]} -gt 0 ] && echo "Echecs/ignores : ${ko[*]}"
# Failure (exit 1) ONLY when no robot succeeded.
[ ${#ok[@]} -eq 0 ] && exit 1
exit 0
