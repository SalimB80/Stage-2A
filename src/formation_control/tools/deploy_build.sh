#!/bin/bash
# deploy_build.sh — copie le package sur chaque robot puis build a distance.
#
# Robuste : un robot injoignable est IGNORE proprement (ne fait plus echouer
# tout le deploiement, contrairement a l'ancien 'set -e'). Cree l'arborescence
# manquante avant la copie. Affiche un RESUME clair par robot a la fin -> tu
# vois d'un coup d'oeil lesquels ont bien recu le nouveau code.
#
#   ./deploy_build.sh 2 3      -> tortuga2 et tortuga3
#   ./deploy_build.sh          -> tente 1 2 3 4

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

  # build ; on capture la sortie pour diagnostiquer un echec eventuel
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
# Echec (code 1) UNIQUEMENT si aucun robot n'a reussi.
[ ${#ok[@]} -eq 0 ] && exit 1
exit 0
