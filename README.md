# Système de localisation BLE + Fusion Kalman + SLAM

Localisation d'un robot mobile en intérieur par trilatération BLE (3 ancres ESP32),
fusionnée avec l'odométrie via un filtre de Kalman, comparée en parallèle avec un SLAM.

---

## Table des matières

1. [Architecture du système](#1-architecture-du-système)
2. [Matériel requis](#2-matériel-requis)
3. [Configuration partagée — shared_config.h](#3-configuration-partagée--shared_configh)
4. [Flash des ESP32 ancres](#4-flash-des-esp32-ancres)
5. [Flash de l'ESP32 robot](#5-flash-de-lesp32-robot)
6. [Démarrage du broker MQTT](#6-démarrage-du-broker-mqtt)
7. [Calibration du modèle BLE](#7-calibration-du-modèle-ble)
8. [Compilation des packages ROS 2](#8-compilation-des-packages-ros-2)
9. [Lancement du SLAM](#9-lancement-du-slam)
10. [Lancement du système de localisation BLE](#10-lancement-du-système-de-localisation-ble)
11. [Visualisation dans RViz](#11-visualisation-dans-rviz)
12. [Paramètres à ajuster](#12-paramètres-à-ajuster)
13. [Dépannage](#13-dépannage)

---

## 1. Architecture du système

```
[ESP32 Ancre 1/2/3]  →  BLE advertising
        ↓
[ESP32 Robot]        →  scan BLE + moyenne RSSI  →  MQTT (WiFi)
        ↓
[Broker Mosquitto]   →  topic PDR/robot/rssi
        ↓
[trilateration_node] →  EMA RSSI → distances → least_squares → ble_estimated_position
        ↓
[fusion_node]        →  Kalman (odom TF + BLE) → fused_pose (dans ble_origin = map)
        ↓
[RViz]               →  comparaison fused_pose ↔ SLAM pose
```

**Arbre TF complet :**
```
map ──(static identity)──► ble_origin
 │
 └──(SLAM)──► odom ──► base_footprint
```

---

## 2. Matériel requis

| Élément | Quantité | Remarque |
|---|---|---|
| ESP32 (modèle avec BLE) | 4 | 3 ancres + 1 robot |
| Câbles USB | 4 | Flash et alimentation |
| Réseau WiFi local | 1 | Même réseau que le PC ROS |
| Machine Ubuntu | 1 | ROS 2 + broker MQTT |

---

## 3. Configuration partagée — shared_config.h

Ce fichier est inclus par `main_ancre.cpp` et `main_robot.cpp`.
Il doit être placé dans le dossier `include/` de chaque projet PlatformIO.

```cpp
// shared_config.h
#pragma once

// --- WiFi ---
const char* ssid     = "VOTRE_SSID";
const char* password = "VOTRE_MOT_DE_PASSE";

// --- Broker MQTT ---
const char* mqtt_server = "10.120.2.231";  // IP Ubuntu : vérifier avec hostname -I
const int   mqtt_port   = 1883;

// --- Noms des ancres BLE ---
// Doivent correspondre exactement aux noms émis par main_ancre.cpp
// Format généré automatiquement : "ESP32_Anchor_ID:" + ANCHOR_ID
#define ANCHOR_1_NAME "ESP32_Anchor_ID:1"
#define ANCHOR_2_NAME "ESP32_Anchor_ID:2"
#define ANCHOR_3_NAME "ESP32_Anchor_ID:3"
```

> **Important :** mettre à jour `IP_ADRESS` dans `trilateration_node.py`
> avec la même IP que `mqtt_server`. Vérifier avec `hostname -I` depuis Ubuntu.

---

## 4. Flash des ESP32 ancres

Chaque ancre doit avoir un `ANCHOR_ID` unique (1, 2 ou 3).
Le nom BLE émis est automatiquement `"ESP32_Anchor_ID:<ID>"`.

### Procédure pour chaque ancre

**Étape 1 — Modifier l'ID dans `main_ancre.cpp` (ligne 12)**

```cpp
#define ANCHOR_ID 1   // → 1, 2 ou 3 selon l'ancre flashée
```

**Étape 2 — Flasher via PlatformIO**

```bash
pio run --target upload --upload-port /dev/ttyUSB0
# Lister les ports disponibles si nécessaire :
ls /dev/ttyUSB* /dev/ttyACM*
```

**Étape 3 — Vérifier dans le moniteur série**

```bash
pio device monitor --baud 115200
```

Sortie attendue :
```
§§§-----------------------------§§§
Emitting BLE as : ESP32_Anchor_ID:1
§§§-----------------------------§§§
```

La LED bleue clignote à 1 Hz quand l'ancre émet correctement.

**Étape 4 — Répéter pour ANCHOR_ID 2 et 3**

### Placement physique des ancres

Coordonnées à partir de l'origine (point 0 marqué au sol, robot face à X) :

| Ancre | Nom BLE | X (m) | Y (m) | Z (m) |
|---|---|---|---|---|
| Anchor_1 | ESP32_Anchor_ID:1 | 2.36 | 5.92 | 1.08 |
| Anchor_2 | ESP32_Anchor_ID:2 | 9.94 | 0.25 | 0.89 |
| Anchor_3 | ESP32_Anchor_ID:3 | 0.60 | 1.50 | 1.1  |

Fixez les ancres de manière rigide. Tout déplacement après calibration invalide les mesures.

---

## 5. Flash de l'ESP32 robot

L'ESP32 embarqué sur le robot scanne les ancres BLE et publie les RSSI moyennés via MQTT.

```bash
# Depuis le projet PlatformIO main_robot
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --baud 115200
```

Sortie attendue à chaque cycle (~200 ms) :
```
Scan BLE (200ms)...
Anchor_1: -62 dBm (4 samples) | Anchor_2: -75 dBm (2 samples) | Anchor_3: -58 dBm (6 samples)
Publication MQTT : {"Anchor_1":-62,"Anchor_2":-75,"Anchor_3":-58}
```

> Si une ancre affiche `0 samples`, elle n'est pas vue pendant le scan.
> Vérifiez qu'elle émet (LED clignote) et qu'elle est à portée.

---

## 6. Démarrage du broker MQTT

Le broker Mosquitto doit tourner sur la machine Ubuntu **avant tout lancement ROS**.

### Installation (première fois seulement)

```bash
sudo apt install mosquitto mosquitto-clients
```

### Démarrage

```bash
sudo systemctl start mosquitto
sudo systemctl enable mosquitto   # démarrage automatique au boot
```

### Configuration — autoriser les connexions externes

```bash
sudo nano /etc/mosquitto/mosquitto.conf
```

Ajouter ou vérifier la présence de ces deux lignes :
```
listener 1883
allow_anonymous true
```

```bash
sudo systemctl restart mosquitto
sudo ufw allow 1883   # si le pare-feu est actif
```

### Vérification — écouter les données brutes

```bash
mosquitto_sub -h 10.120.2.231 -p 1883 -t "PDR/robot/rssi"
```

Vous devez voir des JSON arriver toutes les ~200 ms :
```json
{"Anchor_1":-62,"Anchor_2":-75,"Anchor_3":-58}
```

---

## 7. Calibration du modèle BLE

Le modèle log-distance convertit RSSI → distance :

```
distance = 10 ^ ((A_CONST - RSSI) / (10 × N_CONST))
```

### Calibration de A_CONST (RSSI à 1 mètre)

1. Placez le robot à **exactement 1 mètre** d'une ancre, en vue directe.
2. Collectez 50 mesures et calculez la médiane :

```bash
mosquitto_sub -h 10.120.2.231 -p 1883 -t "PDR/robot/rssi" -C 50 | \
  python3 -c "
import sys, json, statistics
vals = [json.loads(l)['Anchor_1'] for l in sys.stdin if json.loads(l)['Anchor_1'] != -100]
print('Médiane RSSI à 1m :', statistics.median(vals), 'dBm')
"
```

3. Remplacez `A_CONST` dans `trilateration_node.py` par cette valeur.
4. Répétez pour les 3 ancres. Si les valeurs diffèrent de plus de 3 dBm, utilisez un dictionnaire par ancre.

### Calibration de N_CONST (exposant de perte de trajet)

**C'est le paramètre le plus impactant sur la précision.**

1. Placez le robot à **1m, 2m, 3m et 4m** d'une ancre.
2. Relevez le RSSI moyen (20 mesures) à chaque distance.
3. Calculez N pour chaque point :

```
N = (A_CONST - RSSI_mesuré) / (10 × log10(distance_réelle))
```

4. Faites la moyenne des valeurs obtenues → c'est votre `N_CONST`.

**Valeurs typiques :**

| Environnement | N_CONST |
|---|---|
| Espace ouvert | 2.0 – 2.5 |
| Bureau / couloir dégagé | 2.5 – 3.0 |
| Salle avec obstacles | 3.0 – 3.5 |
| Environnement très réfléchissant | 3.5 – 4.5 |

**Vérification en live après modification :**

```bash
ros2 launch sensor_fusion localization_launch.py
```

Les logs affichent pour chaque cycle :
```
Anchor_1: -62.3dBm → 2.14m | Anchor_2: -74.8dBm → 5.31m | Anchor_3: -59.1dBm → 1.87m
Position estimée → X=3.21m, Y=2.87m
```

Comparez les distances affichées avec les distances réelles mesurées au sol.

---

## 8. Compilation des packages ROS 2

### Prérequis

```bash
# ROS 2 Humble (ou Iron)
source /opt/ros/humble/setup.bash

# Dépendances Python
pip install paho-mqtt scipy numpy
```

### Structure des packages

```
ros2_ws/
└── src/
    ├── ble_localization/
    │   ├── ble_localization/
    │   │   └── trilateration_node.py
    │   ├── setup.py
    │   └── package.xml
    └── sensor_fusion/
        ├── sensor_fusion/
        │   └── fusion_node.py
        ├── launch/
        │   └── localization_launch.py
        ├── setup.py
        └── package.xml
```

### Compilation

```bash
cd ~/ros2_ws
colcon build --packages-select ble_localization sensor_fusion
source install/setup.bash
```

> Pour éviter de recompiler à chaque modification Python pendant le développement :
> ```bash
> colcon build --packages-select ble_localization sensor_fusion --symlink-install
> ```

---

## 9. Lancement du SLAM

Lancez le SLAM **en premier** — il doit publier la TF `map → odom → base_footprint`
avant le système BLE.

```bash
# Terminal 1 — bringup du robot (drivers, odométrie, LIDAR)
ros2 launch <votre_robot_package> robot_bringup.launch.py

# Terminal 2 — SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=<chemin>/mapper_params_online_async.yaml
```

### Vérification de l'arbre TF

```bash
ros2 run tf2_tools view_frames
# Ouvre frames.pdf — vérifier la chaîne : map → odom → base_footprint
```

Ou en ligne de commande :
```bash
ros2 topic echo /tf --once | grep frame_id
```

Les frames `map`, `odom` et `base_footprint` doivent être présentes avant de continuer.

---

## 10. Lancement du système de localisation BLE

Une fois le broker MQTT actif et le SLAM démarré :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch sensor_fusion localization_launch.py
```

Ce launch démarre 3 nœuds :

| Nœud | Package | Rôle |
|---|---|---|
| `static_transform_publisher` | tf2_ros | Publie `map → ble_origin` (identité) |
| `ble_trilateration_node` | ble_localization | RSSI → position BLE brute |
| `kalman_fusion_node` | sensor_fusion | Fusion odom + BLE → fused_pose |

### Topics publiés

| Topic | Type | Description |
|---|---|---|
| `/ble_estimated_position` | `PoseWithCovarianceStamped` | Position BLE brute dans `ble_origin` |
| `/fused_pose` | `PoseStamped` | Position fusionnée Kalman dans `ble_origin` |

---

## 11. Visualisation dans RViz

```bash
rviz2
```

**Fixed Frame :** `map`

### Displays à ajouter

| Display | Topic | Couleur suggérée |
|---|---|---|
| Map | `/map` | — |
| RobotModel | — | — |
| PoseWithCovarianceStamped | `/ble_estimated_position` | Bleu — BLE brut |
| PoseStamped | `/fused_pose` | Vert — Kalman BLE+odom |
| Pose SLAM | topic selon votre SLAM | Rouge — référence |
| TF | 'base_footprint' | Comparer avec le slam, debug |

Les trois poses s'affichent toutes dans le repère `map` sans conflit,
permettant une comparaison directe SLAM vs BLE fusionné.

---

## 12. Paramètres à ajuster

### `trilateration_node.py`

```python
IP_ADRESS   = '10.120.2.231'  # IP du broker MQTT (hostname -I sur Ubuntu)
A_CONST     = -40             # RSSI à 1m — calibrer (section 7)
N_CONST     = 3.8             # Exposant de perte — calibrer (section 7)
EMA_ALPHA   = 0.2             # Lissage RSSI : 0.1 (fort) ↔ 0.5 (réactif)
MIN_ANCHORS = 3               # Ancres minimum pour publier une position
```

### `fusion_node.py`

```python
self.Q = np.eye(2) * 0.05  # Bruit processus : augmenter = plus de confiance à l'odom
self.R = np.eye(2) * 4.0   # Bruit mesure BLE : augmenter = moins de confiance au BLE
```

**Réglage Q / R :**

| Symptôme | Action |
|---|---|
| Position saute trop sur les mesures BLE | Augmenter R |
| Position dérive trop lors des déplacements | Diminuer R ou augmenter Q |
| Filtre trop lent à corriger une erreur | Diminuer R |

### `main_robot.cpp`

```cpp
const int SCAN_TIME   = 200;  // Durée du scan BLE en ms — augmenter si ancres manquées
const int MAX_SAMPLES = 10;   // Échantillons max par ancre par scan
```

---

## 13. Dépannage

### Position BLE figée ou ne bouge pas

- Vérifier que 3 ancres sont visibles dans MQTT : `mosquitto_sub -t "PDR/robot/rssi"` doit montrer des valeurs ≠ -100 pour les 3 ancres.
- Vérifier les logs du node : `ros2 topic echo /ble_estimated_position`

### Offset permanent entre fused_pose et position réelle

- S'assurer que le robot démarre au point 0 marqué au sol face à X.
- Vérifier que `localization_launch.py` utilise bien `map` (et non `odom`) comme parent de `ble_origin`.
- Recalibrer `A_CONST` (section 7).

### Erreur TF "would result in a loop"

- Le broadcaster TF dans `fusion_node.py` doit rester **commenté** quand SLAM tourne.
- Vérifier dans le launch que le parent est `map` et non `odom`.

### Une ancre affiche toujours -100 dans MQTT

- Vérifier que l'ancre est alimentée (LED clignote à 1 Hz).
- Augmenter `SCAN_TIME` à 400 ms dans `main_robot.cpp`.
- Vérifier la concordance des noms : `ANCHOR_X_NAME` dans `shared_config.h` doit correspondre exactement à `"ESP32_Anchor_ID:<X>"` émis par l'ancre.

### Le broker MQTT refuse les connexions

```bash
sudo nano /etc/mosquitto/mosquitto.conf
# Vérifier la présence de :
# listener 1883
# allow_anonymous true
sudo systemctl restart mosquitto
sudo ufw allow 1883
```

### Compilation colcon échoue

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ble_localization sensor_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
```
