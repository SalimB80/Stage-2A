for i in 1 2 3; do
  sshpass -p 1234 ssh -o StrictHostKeyChecking=no tortuga$i@192.168.0.20$i "mkdir -p ~/formation_ws/src"
done
echo "dossiers crees"
