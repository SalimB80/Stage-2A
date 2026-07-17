# formation_control

Leader-follower de formations pour une flotte de TurtleBot3 Burger (ROS 2).

Le robot **leader** (tortuga1) porte un **casque rectangulaire cyan**. Les
**followers** (tortuga2..4) le detectent a la camera (bearing) et fusionnent
avec le lidar (distance) pour maintenir un offset (range, bearing) propre a
chaque formation. Le systeme s'adapte automatiquement a 2, 3 ou 4 robots.

## Materiel / contexte

- 4x TurtleBot3 Burger : `tortugaX@192.168.0.20X` (X = 1..4), mdp `1234`
- Lidar + camera sur chaque robot
- PC sous Windows + WSL, `ROS_DOMAIN_ID=30`

## Arborescence

```
formation_control/
  formation_control/      # code Python (noeud follower, vision, formations)
  launch/                 # leader.launch.py, follower.launch.py
  config/                 # parametres + profil DDS unicast
  tools/                  # GUI Tkinter + calibration HSV
  scripts/ttb.sh          # script unique de controle de la flotte
```

## Installation (PC WSL)

```bash
sudo apt install sshpass rsync tmux
mkdir -p ~/formation_ws/src
# placer ce dossier dans ~/formation_ws/src/formation_control
cd ~/formation_ws
colcon build --symlink-install
source install/setup.bash
chmod +x src/formation_control/scripts/ttb.sh
```

## Reseau WSL

WSL2 est derriere un NAT : active le mode mirrored dans `C:\Users\<toi>\.wslconfig` :

```ini
[wsl2]
networkingMode=mirrored
```

Puis `wsl --shutdown` (PowerShell admin) et relance WSL. Verifie `ping 192.168.0.201`.

Si le multicast DDS ne passe pas, source le profil unicast partout :

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/formation_ws/src/formation_control/config/fastdds_unicast.xml
```

## Le script ttb.sh

Une seule commande pour tout. `N` = nombre de robots (defaut 4).

```bash
./ttb.sh deploy  [N]              # copie + build le package sur N robots
./ttb.sh start   [N] [formation]  # lance bringup + followers (arriere-plan)
./ttb.sh teleop                   # pilote le leader au clavier
./ttb.sh formation [N] [form]     # change la formation a chaud
./ttb.sh monitor [N]              # grille tmux des logs des robots
./ttb.sh stop    [N]              # arrete tous les noeuds
```

### Workflow typique

```bash
./ttb.sh deploy 4               # une fois (et apres chaque modif du code)
./ttb.sh start 4 triangle       # demarre la flotte
./ttb.sh teleop                 # pilote tortuga1 ; les autres suivent
./ttb.sh formation 4 carre      # change de formation sans tout relancer
./ttb.sh stop 4                 # tout arreter
```

Avec 2 ou 3 robots : remplace simplement `4` par `2` ou `3` partout.

## Calibration du casque cyan (a faire une fois)

L'eclairage de la salle decale la teinte. Sur un robot (ou un PC avec webcam) :

```bash
python3 tools/calibrate_hsv.py
```

Ajuste les sliders jusqu'a isoler le casque, puis reporte les valeurs dans
`formation_control/detector.py` (`CYAN_LOW`, `CYAN_HIGH`). Redeploie ensuite.

## GUI (optionnel)

```bash
python3 tools/launcher_gui.py
```

Selectionne le nombre de robots + la formation, puis Lancer / Changer / Teleop / STOP.

## Dataset : videos + rosbag (.db3)

Le mode dataset enregistre, par robot :

- des **segments video** (`tortugaX_<session>_segNN/` de JPEG + CSV), assembles
  en mp4 par `assemble_video.sh` ;
- un **rosbag** `bag_<date>_tortugaX` (fichier `.db3`, base SQLite) contenant les
  topics `scan` (LaserScan), `odom` (Odometry) et `imu` (Imu).

Un `.db3` n'est pas lisible tel quel. `tools/dataset_tools.sh collect` fait
maintenant TOUT automatiquement apres le rapatriement :

```bash
cd tools
./dataset_tools.sh collect 1 2 3 4
```

1. rapatrie `~/dataset/` de chaque robot dans `./dataset_collected/tortugaX/` ;
2. assemble chaque segment en `..._segNN.mp4` ;
3. **colle** les segments d'une meme session en `tortugaX_<session>_final.mp4` ;
4. **convertit** chaque rosbag `.db3` en CSV (un par topic :
   `bag_..._scan.csv`, `_odom.csv`, `_imu.csv`), poses **dans le meme dossier**
   que les videos.

Ces deux dernieres etapes sont aussi disponibles seules :

```bash
./dataset_tools.sh concat 1        # (re)colle les segments -> *_final.mp4
./dataset_tools.sh bag2csv 1       # convertit les .db3 rapatries -> CSV
```

Outils sous-jacents (utilisables a la main) :

```bash
# colle tous les segments d'un dossier (ou d'une seule session) :
./concat_segments.sh ./dataset_collected/tortuga1 [tortuga1_20260716_175709]

# convertit un rosbag (fichier .db3 OU dossier de bag) en CSV :
./bag_to_csv.py bag_20260716_175706_tortuga1_0.db3 [dossier_de_sortie]
```

`bag_to_csv.py` utilise ROS 2 (rclpy) s'il est source (n'importe quel type de
message) et retombe sinon sur un decodeur CDR interne pour scan/odom/imu — aucune
dependance ROS necessaire sur le PC. Chaque ligne CSV porte `bag_time_ns` et le
`stamp_sec`/`stamp_nanosec` du header, directement alignables avec les CSV du
recorder (meme horloge ROS).

## Formations disponibles

`colonne`, `ligne`, `triangle`, `carre`. Les offsets (range, bearing) par
follower sont definis dans `formation_control/formations.py` — modifiables.

## Limites connues

- Vision pure : un follower doit voir le casque. Formations profondes (4e robot
  cache derriere les autres) fragiles -> envisager le chainage F->F.
- Un seul casque = un seul leader visible ; les followers ne se distinguent pas
  entre eux. Pour des formations laterales strictes, des AprilTags distincts
  sont plus robustes.
- Tourner le noeud follower SUR le robot (jamais streamer l'image au PC).
```

## Validation 0 -> 100%

1. Calibration cyan sur image reelle.
2. Detection seule (bearing/range affiches, robot immobile).
3. Un follower suit le leader pousse a la main (colonne).
4. Reglage des gains k_lin / k_ang.
5. 2 robots, puis 3, puis 4.
6. Toutes les formations + transitions a chaud.
