# BLE Localization System + Kalman Fusion + SLAM

Indoor localization of a mobile robot using BLE trilateration (3 ESP32 anchors),
fused with wheel odometry via a Kalman filter, compared in parallel with a SLAM.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Required Hardware](#2-required-hardware)
3. [Shared Configuration — shared_config.h](#3-shared-configuration--shared_configh)
4. [Flashing the ESP32 Anchors](#4-flashing-the-esp32-anchors)
5. [Flashing the Robot ESP32](#5-flashing-the-robot-esp32)
6. [Starting the MQTT Broker](#6-starting-the-mqtt-broker)
7. [BLE Model Calibration](#7-ble-model-calibration)
8. [Building the ROS 2 Packages](#8-building-the-ros-2-packages)
9. [Launching the SLAM](#9-launching-the-slam)
10. [Launching the BLE Localization System](#10-launching-the-ble-localization-system)
11. [Visualization in RViz](#11-visualization-in-rviz)
12. [Tunable Parameters](#12-tunable-parameters)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Architecture

```
[ESP32 Anchor 1/2/3]  →  BLE advertising
        ↓
[ESP32 Robot]         →  BLE scan + averaged RSSI  →  MQTT (WiFi)
        ↓
[Mosquitto Broker]    →  topic PDR/robot/rssi
        ↓
[trilateration_node]  →  EMA RSSI → distances → least_squares → ble_estimated_position
        ↓
[fusion_node]         →  Kalman (odom TF + BLE) → fused_pose (in ble_origin = map)
        ↓
[RViz]                →  comparison fused_pose ↔ SLAM pose
```

**Full TF tree:**
```
map ──(static identity)──► ble_origin
 │
 └──(SLAM)──► odom ──► base_footprint
```

---

## 2. Required Hardware

| Item | Quantity | Notes |
|---|---|---|
| ESP32 (BLE-capable model) | 4 | 3 anchors + 1 on the robot |
| USB cables | 4 | Flashing and power |
| Local WiFi network | 1 | Same network as the ROS PC |
| Ubuntu machine | 1 | ROS 2 + MQTT broker |

---

## 3. Shared Configuration — shared_config.h

This file is included by both `main_ancre.cpp` and `main_robot.cpp`.
It must be placed in the `include/` folder of each PlatformIO project.

```cpp
// shared_config.h
#pragma once

// --- WiFi ---
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// --- MQTT Broker ---
const char* mqtt_server = "10.120.2.231";  // Ubuntu IP — check with: hostname -I
const int   mqtt_port   = 1883;

// --- BLE Anchor Names ---
// Must match exactly the names emitted by main_ancre.cpp
// Auto-generated format: "ESP32_Anchor_ID:" + ANCHOR_ID
#define ANCHOR_1_NAME "ESP32_Anchor_ID:1"
#define ANCHOR_2_NAME "ESP32_Anchor_ID:2"
#define ANCHOR_3_NAME "ESP32_Anchor_ID:3"
```

> **Important:** update `IP_ADRESS` in `trilateration_node.py` with the same IP
> as `mqtt_server`. Check it with `hostname -I` from the Ubuntu terminal.

---

## 4. Flashing the ESP32 Anchors

Each anchor must have a unique `ANCHOR_ID` (1, 2, or 3).
The emitted BLE name is automatically `"ESP32_Anchor_ID:<ID>"`.

### Procedure for each anchor

**Step 1 — Edit the ID in `main_ancre.cpp` (line 12)**

```cpp
#define ANCHOR_ID 1   // → 1, 2, or 3 depending on the anchor being flashed
```

**Step 2 — Flash via PlatformIO**

```bash
pio run --target upload --upload-port /dev/ttyUSB0
# List available ports if needed:
ls /dev/ttyUSB* /dev/ttyACM*
```

**Step 3 — Verify in the serial monitor**

```bash
pio device monitor --baud 115200
```

Expected output:
```
§§§-----------------------------§§§
Emitting BLE as : ESP32_Anchor_ID:1
§§§-----------------------------§§§
```

The blue LED blinks at 1 Hz when the anchor is broadcasting correctly.

**Step 4 — Repeat for ANCHOR_ID 2 and 3**

### Physical placement of anchors

Coordinates from the origin (point 0 marked on the floor, robot facing X):

| Anchor | BLE Name | X (m) | Y (m) |
|---|---|---|---|
| Anchor_1 | ESP32_Anchor_ID:1 | 2.36 | 5.92 |
| Anchor_2 | ESP32_Anchor_ID:2 | 9.94 | 0.25 |
| Anchor_3 | ESP32_Anchor_ID:3 | 0.60 | 1.50 |

Mount anchors rigidly. Any movement after calibration invalidates measurements.

---

## 5. Flashing the Robot ESP32

The ESP32 mounted on the robot scans the BLE anchors and publishes averaged RSSI values via MQTT.

```bash
# From the main_robot PlatformIO project
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --baud 115200
```

Expected output every cycle (~200 ms):
```
Scan BLE (200ms)...
Anchor_1: -62 dBm (4 samples) | Anchor_2: -75 dBm (2 samples) | Anchor_3: -58 dBm (6 samples)
Publication MQTT : {"Anchor_1":-62,"Anchor_2":-75,"Anchor_3":-58}
```

> If an anchor shows `0 samples`, it is not visible during the scan window.
> Check that it is broadcasting (LED blinking) and within range.

---

## 6. Starting the MQTT Broker

The Mosquitto broker must be running on the Ubuntu machine **before any ROS launch**.

### Installation (first time only)

```bash
sudo apt install mosquitto mosquitto-clients
```

### Start the broker

```bash
sudo systemctl start mosquitto
sudo systemctl enable mosquitto   # auto-start on boot
```

### Configuration — allow external connections

```bash
sudo nano /etc/mosquitto/mosquitto.conf
```

Add or verify the presence of these two lines:
```
listener 1883
allow_anonymous true
```

```bash
sudo systemctl restart mosquitto
sudo ufw allow 1883   # if the firewall is active
```

### Verification — listen to raw data

```bash
mosquitto_sub -h 10.120.2.231 -p 1883 -t "PDR/robot/rssi"
```

You should see JSON packets arriving every ~200 ms:
```json
{"Anchor_1":-62,"Anchor_2":-75,"Anchor_3":-58}
```

---

## 7. BLE Model Calibration

The log-distance model converts RSSI → distance:

```
distance = 10 ^ ((A_CONST - RSSI) / (10 × N_CONST))
```

### Calibrating A_CONST (RSSI at 1 meter)

1. Place the robot at **exactly 1 meter** from an anchor, in direct line of sight.
2. Collect 50 measurements and compute the median:

```bash
mosquitto_sub -h 10.120.2.231 -p 1883 -t "PDR/robot/rssi" -C 50 | \
  python3 -c "
import sys, json, statistics
vals = [json.loads(l)['Anchor_1'] for l in sys.stdin if json.loads(l)['Anchor_1'] != -100]
print('Median RSSI at 1m:', statistics.median(vals), 'dBm')
"
```

3. Replace `A_CONST` in `trilateration_node.py` with this value.
4. Repeat for all 3 anchors. If values differ by more than 3 dBm between anchors, use a per-anchor dictionary.

### Calibrating N_CONST (path loss exponent)

**This is the most impactful parameter for accuracy.**

1. Place the robot at **1m, 2m, 3m, and 4m** from an anchor.
2. Record the average RSSI (20 measurements) at each distance.
3. Compute N for each point:

```
N = (A_CONST - measured_RSSI) / (10 × log10(real_distance))
```

4. Average the values obtained → this is your `N_CONST`.

**Typical values by environment:**

| Environment | N_CONST |
|---|---|
| Open space | 2.0 – 2.5 |
| Office / clear corridor | 2.5 – 3.0 |
| Room with obstacles | 3.0 – 3.5 |
| Highly reflective environment | 3.5 – 4.5 |

**Live verification after updating the value:**

```bash
ros2 launch sensor_fusion localization_launch.py
```

Logs display for each cycle:
```
Anchor_1: -62.3dBm → 2.14m | Anchor_2: -74.8dBm → 5.31m | Anchor_3: -59.1dBm → 1.87m
Position estimated → X=3.21m, Y=2.87m
```

Compare displayed distances against real distances measured on the floor.

---

## 8. Building the ROS 2 Packages

### Prerequisites

```bash
# ROS 2 Humble (or Iron)
source /opt/ros/humble/setup.bash

# Python dependencies
pip install paho-mqtt scipy numpy
```

### Package structure

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

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select ble_localization sensor_fusion
source install/setup.bash
```

> To avoid rebuilding after every Python file edit during development:
> ```bash
> colcon build --packages-select ble_localization sensor_fusion --symlink-install
> ```

---

## 9. Launching the SLAM

Launch the SLAM **first** — it must publish the TF chain `map → odom → base_footprint`
before the BLE system starts.

```bash
# Terminal 1 — robot bringup (drivers, odometry, LiDAR)
ros2 launch <your_robot_package> robot_bringup.launch.py

# Terminal 2 — SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=<path>/mapper_params_online_async.yaml
```

### Verify the TF tree

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf — verify the chain: map → odom → base_footprint
```

Or from the command line:
```bash
ros2 topic echo /tf --once | grep frame_id
```

The frames `map`, `odom`, and `base_footprint` must all be present before continuing.

---

## 10. Launching the BLE Localization System

Once the MQTT broker is running and the SLAM is started:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch sensor_fusion localization_launch.py
```

This launch starts 3 nodes:

| Node | Package | Role |
|---|---|---|
| `static_transform_publisher` | tf2_ros | Publishes `map → ble_origin` (identity) |
| `ble_trilateration_node` | ble_localization | RSSI → raw BLE position |
| `kalman_fusion_node` | sensor_fusion | Fuses odom + BLE → fused_pose |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `/ble_estimated_position` | `PoseWithCovarianceStamped` | Raw BLE position in `ble_origin` |
| `/fused_pose` | `PoseStamped` | Kalman-fused position in `ble_origin` |

---

## 11. Visualization in RViz

```bash
rviz2
```

**Fixed Frame:** `map`

### Displays to add

| Display | Topic | Suggested color |
|---|---|---|
| Map | `/map` | — |
| RobotModel | — | — |
| PoseWithCovarianceStamped | `/ble_estimated_position` | Blue — raw BLE |
| PoseStamped | `/fused_pose` | Green — Kalman BLE+odom |
| SLAM Pose | topic depends on your SLAM | Red — reference |
| TF | — | Optional, for debug |

All three poses are displayed in the `map` frame without conflict,
enabling direct comparison between SLAM and fused BLE.

---

## 12. Tunable Parameters

### `trilateration_node.py`

```python
IP_ADRESS   = '10.120.2.231'  # MQTT broker IP (hostname -I on Ubuntu)
A_CONST     = -40             # RSSI at 1m — calibrate (section 7)
N_CONST     = 3.8             # Path loss exponent — calibrate (section 7)
EMA_ALPHA   = 0.2             # RSSI smoothing: 0.1 (heavy) ↔ 0.5 (reactive)
MIN_ANCHORS = 3               # Minimum anchors required to publish a position
```

### `fusion_node.py`

```python
self.Q = np.eye(2) * 0.05  # Process noise: increase = trust odometry more
self.R = np.eye(2) * 4.0   # Measurement noise: increase = trust BLE less
```

**Q / R tuning guide:**

| Symptom | Action |
|---|---|
| Position jumps too much on BLE measurements | Increase R |
| Position drifts too much during movement | Decrease R or increase Q |
| Filter too slow to correct an error | Decrease R |

### `main_robot.cpp`

```cpp
const int SCAN_TIME   = 200;  // BLE scan duration in ms — increase if anchors are missed
const int MAX_SAMPLES = 10;   // Max samples per anchor per scan
```

---

## 13. Troubleshooting

### BLE position is frozen or does not move

- Check that all 3 anchors are visible in MQTT: `mosquitto_sub -t "PDR/robot/rssi"` should show values ≠ -100 for all 3 anchors.
- Check node logs: `ros2 topic echo /ble_estimated_position`

### Permanent offset between fused_pose and actual position

- Make sure the robot starts at the marked point 0 on the floor, facing X.
- Check that `localization_launch.py` uses `map` (not `odom`) as the parent of `ble_origin`.
- Recalibrate `A_CONST` (section 7).

### TF error "would result in a loop"

- The TF broadcaster in `fusion_node.py` must remain **commented out** when SLAM is running.
- Verify in the launch file that the parent frame is `map` and not `odom`.

### An anchor always shows -100 in MQTT

- Check the anchor is powered (LED blinking at 1 Hz).
- Increase `SCAN_TIME` to 400 ms in `main_robot.cpp`.
- Check name consistency: `ANCHOR_X_NAME` in `shared_config.h` must exactly match `"ESP32_Anchor_ID:<X>"` emitted by the anchor.

### MQTT broker refuses connections

```bash
sudo nano /etc/mosquitto/mosquitto.conf
# Verify the presence of:
# listener 1883
# allow_anonymous true
sudo systemctl restart mosquitto
sudo ufw allow 1883
```

### colcon build fails

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ble_localization sensor_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
```
