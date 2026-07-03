# IMU_Reconstruction

**Real‑time Human Motion Capture using ESP32‑C3, MPU6050, and a Custom C++ Visualizer**

[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
---

## Overview

This project implements a complete, low‑latency **inertial motion capture** system. It uses a network of **ESP32‑based sensor nodes** (each with an MPU6050 IMU) that communicate wirelessly via **ESP‑NOW** to one or more **hub nodes**, which forward the data via **UDP multicast** to a PC. A custom **C++/OpenGL** visualizer receives the data, applies calibration, filtering, and inverse‑kinematics‑style bone mapping, and animates a 3D human model (loaded from a `human.glb` file) in real time.

The system supports up to **10 tracked body segments**:  
*Left/Right Upper Arm, Left/Right Forearm, Left/Right Thigh, Left/Right Shin, Hips, and Chest.*  
All sensors are battery‑powered and wireless, making the setup suitable for untethered motion capture.

---

## Features

- **Wireless sensor network** based on ESP‑NOW with **automatic channel discovery** – sensors self‑configure to the hub’s current WiFi channel.
- **Low latency** – sensor data is sent at >60 Hz; visualizer updates at display framerate.
- **Calibration** – per‑sensor neutral pose capture; supports 8 quaternion conventions (horizontal/vertical mounting, axis remapping) to handle arbitrary sensor orientations.
- **Robust smoothing & drift compensation** – adaptive SLERP filtering and automatic drift correction when the sensor is stationary.
- **Hierarchical bone mapping** – sensors drive a skeleton in local (parent‑relative) space, maintaining correct joint orientations.
- **CSV logging** – automatically logs all joint rotations (quaternions) to timestamped CSV files for post‑processing.
- **Placement guide overlay** – visual markers on the 3D model show recommended sensor positions.
- **Cross‑platform** – runs on Linux (requires GLFW, OpenGL, GLUT).

---

## Hardware Requirements

- **Sensor nodes:** ESP32‑based boards (e.g., ESP32‑C3, ESP32‑S2, or classic ESP32) with an **MPU6050** 6‑axis IMU connected via I²C.
- **Hub node:** One ESP32 board (acts as a central receiver and forwards data to the PC).
- **PC:** Linux machine with OpenGL support.

### Wiring (MPU6050 → ESP32)

| MPU6050 | ESP32 |
|---------|-------|
| VCC     | 3.3V  |
| GND     | GND   |
| SCL     | GPIO9 (or configured in code) |
| SDA     | GPIO8 (or configured in code) |
| INT     | Not used (DMP FIFO read is polled) |

*Note: The I²C pins in the provided code are `8` (SDA) and `9` (SCL). Adjust if your board uses different pins.*

### Sensor Labels & Placement

Each sensor must be flashed with a unique label (defined in `sensor.ino` via `SENSOR_LABEL`). The labels and recommended body locations are:

| Label | Body Part       |
|-------|-----------------|
| `L_UA`| Left Upper Arm  |
| `R_UA`| Right Upper Arm |
| `L_FA`| Left Forearm    |
| `R_FA`| Right Forearm   |
| `L_TH`| Left Thigh      |
| `R_TH`| Right Thigh     |
| `L_SH`| Left Shin       |
| `R_SH`| Right Shin      |
| `HIPS`| Pelvis          |
| `CHEST`| Torso (sternum)|

**Important:** The sensors must be mounted so that their **LED (or switch) orientation** is consistent per body side. The default mode assumes horizontal mounting with the switch facing up. For vertical mounting (LED up/down), use the `P` key to toggle the vertical mode and cycle through the 8 possible orientation conventions.

---

## Software Requirements

- **C++17** compiler (g++ or clang)
- **GLFW** (≥ 3.3)
- **GLUT** (freeglut or similar)
- **OpenGL** (system provided)
- **GLM** (header‑only math library)
- **cgltf** (included in `third_party/`)

On **Ubuntu/Debian**:
```bash
sudo apt install build-essential libglfw3-dev freeglut3-dev libglm-dev
```

---

## Building

Clone the repository:

```bash
git clone https://github.com/thorOdinson16/IMU_Reconstruction.git
cd IMU_Reconstruction
```

Compile the visualizer:

```bash
g++ -std=c++17 -o imu_visualizer \
    main.cpp sensor_manager.cpp udp_receiver.cpp \
    input_handler.cpp renderer.cpp gltf_model.cpp \
    csv_logger.cpp \
    -lglfw -lGL -lGLU -lglut -lpthread \
    -I/usr/include/glm -Ithird_party
```

*(Adjust library paths if necessary.)*

---

## Running

1. Ensure your PC is connected to the same WiFi network as the hub (the hub connects to your WiFi and forwards sensor data via UDP multicast).
2. Place the `human.glb` file (Mixamo‑compatible skeleton with skinning) in the same directory as the executable.
3. Run the visualizer:
   ```bash
   ./imu_visualizer
   ```
4. Power on the hub and all sensor nodes. The sensors will automatically discover the hub and begin streaming data.
5. Stand in a neutral pose (arms down, legs straight, torso upright) and press the **Spacebar** to calibrate all sensors at once, or calibrate individually using the keys below.

---

## System Architecture

### Sensor Nodes (`sensor.ino`)

- Initialise the MPU6050 DMP and calibrate gyro/accelerometer.
- Run a **channel discovery** routine: sends a `"WHO?"` probe on channels 1‑13, waits for a `"HERE"` reply from the hub, and locks onto the channel where the reply is received.
- After discovery, send `SensorData` (label + quaternion) via ESP‑NOW to the hub at ~60 Hz.

### Hub Node (`hub.ino`)

- Connects to WiFi, reads its current channel, and broadcasts that channel in reply to probe packets.
- Receives ESP‑NOW packets from sensors; forwards them (via UDP multicast) to the PC, along with its own internal MPU6050 orientation (if equipped).
- Supports two hubs: one for upper‑body sensors (CHEST) and one for lower‑body sensors (HIPS). The code provided is for a single hub; you can flash separate hubs with different MAC addresses.

### PC Visualizer (`main.cpp` + modules)

- **UDP Receiver** (`udp_receiver.cpp`) listens on port 5005, parses incoming comma‑separated quaternion strings, and updates the `SensorManager`.
- **SensorManager** (`sensor_manager.cpp`) maintains per‑sensor state: raw quaternion, calibration reference, smoothed corrected quaternion, drift correction, and active flags.
- **GltfModel** (`gltf_model.cpp`) loads the `human.glb` skeleton, computes the hierarchical bone rotations (local space) from the corrected sensor quaternions, and performs the skinning pass.
- **Renderer** (`renderer.cpp`) handles OpenGL viewport, camera controls, lighting, and draws the HUD axis widgets.
- **InputHandler** (`input_handler.cpp`) translates keyboard presses to calibration actions and mode toggles.

### Data Flow

```
Sensor (ESP32) ──ESP‑NOW──> Hub (ESP32) ──UDP multicast──> PC Visualizer
                                                              │
                                                    ┌─────────┴─────────┐
                                                    │ SensorManager     │
                                                    │  - Calibration    │
                                                    │  - Smoothing      │
                                                    │  - Drift corr.    │
                                                    └─────────┬─────────┘
                                                              │
                                                    ┌─────────┴─────────┐
                                                    │ GltfModel         │
                                                    │  - Bone mapping   │
                                                    │  - Skinning       │
                                                    └─────────┬─────────┘
                                                              │
                                                         (Render)
```

---

## Calibration & Controls

Hold the neutral pose (arms down, legs straight, torso upright), then press the appropriate calibration key.  
The active quaternion mode determines how raw sensor data is converted to body motion.

### Calibration Keys

| Key | Action                         |
|-----|--------------------------------|
| `C` | Calibrate **L_FA** (left forearm) |
| `V` | Calibrate **R_FA** (right forearm)|
| `B` | Calibrate **L_UA** (left upper arm)|
| `N` | Calibrate **R_UA** (right upper arm)|
| `Z` | Calibrate **L_TH** (left thigh)|
| `X` | Calibrate **L_SH** (left shin) |
| `G` | Calibrate **R_TH** (right thigh)|
| `H` | Calibrate **R_SH** (right shin)|
| `I` | Calibrate **HIPS** (pelvis)    |
| `O` | Calibrate **CHEST** (torso)    |
| `Space` | Re‑calibrate **all sensors at once** |

### Mode Selection

| Key | Action |
|-----|--------|
| `M` | Cycle through the 4 quaternion conventions (modes 1‑4 or 5‑8, depending on vertical toggle). |
| `P` | Toggle between **horizontal** and **vertical** sensor mounting. |
| `L` | Toggle **placement guide** overlay (shows recommended sensor positions on the model). |

### Camera Views

| Key | Action |
|-----|--------|
| `1` | Front view |
| `2` | Back view  |
| `3` | Side view  |

### Quaternion Modes

The 8 modes handle different physical sensor orientations:

- **Modes 1‑4** (horizontal mounting): press `M` to cycle `1 → 2 → 3 → 4 → 1…`  
- **Modes 5‑8** (vertical mounting): press `P` first (enters mode 5), then `M` to cycle `5 → 6 → 7 → 8 → 5…`

After changing the mode with `M` or toggling vertical with `P`, **you must recalibrate** (individual key or `Space`) for the new setting to take effect.

> **Tip:** Start with mode 2 (horizontal, switch up) or mode 5 (vertical, LED up), then cycle until the motion matches the physical movement. If the limb bends the wrong way or twists unnaturally, try a different mode.

---

## CSV Logging

The visualizer automatically writes a CSV file (`CSV/run_XXX.csv`) after the first sensor data is received and 5 seconds have passed. Each row contains:
- Timestamp (seconds since program start)
- For each tracked joint (in fixed order): `w, x, y, z` quaternion (local rotations for arms/legs, world for hips/chest)
- An optional `event` column for calibration markers.

The log is useful for offline analysis or replay.

---

## Troubleshooting

### Sensors not connecting to hub
- Ensure the hub is powered and connected to WiFi (the green LED on the hub lights up for 2 seconds after connection).
- Check that the sensor’s `HUB_MAC` in `sensor.ino` matches the correct hub MAC address (CHEST or HIPS).
- The discovery process may take up to 4 seconds per sweep; if it fails, it will retry indefinitely.

### Visualizer shows no motion
- Verify the PC is on the same network as the hub and can receive multicast (`239.0.0.1:5005`).
- Run `sudo tcpdump -i any port 5005` to check if UDP packets are arriving.
- Check the console output for “Listening on UDP 5005” and any error messages.

### Model loads but doesn’t move correctly
- Re‑calibrate all sensors in a neutral pose.
- Cycle through the quaternion modes (`M`) and recalibrate until the joint orientations match reality.
- Ensure the sensor labels match the expected labels in `udp_receiver.cpp`.

### Compilation issues
- Make sure GLM is installed and the include path is correct.
- The `cgltf.h` parser is header‑only and should be placed in `third_party/`.

---

## File Structure

```
IMU_Reconstruction/
├── arduino_code/             # ESP32 firmware
│   ├── hub.ino               # Hub node (receives ESP‑NOW, forwards UDP)
│   ├── sensor.ino            # Sensor node (MPU6050 + ESP‑NOW sender)
│   ├── singleUDP.ino         # Optional standalone UDP sender (for testing)
│   └── PROTOCOL_NOTES.md     # ESP‑NOW protocol definition
├── docs/                     # Additional documentation
│   ├── Notes.md              # Quaternion modes explained
│   └── Working.md            # In‑depth explanation of the math
├── third_party/              # External libraries
│   └── cgltf.h               # GLTF/GLB loader (single‑header)
├── csv_logger.cpp/h          # CSV logging
├── gltf_model.cpp/h          # GLTF model loading & skeleton animation
├── input_handler.cpp/h       # Keyboard input handling
├── renderer.cpp/h            # OpenGL rendering & HUD
├── sensor_manager.cpp/h      # Core sensor state, calibration, filtering
├── udp_receiver.cpp/h        # UDP multicast receiver thread
├── main.cpp                  # Program entry point
├── human.glb                 # 3D human model (Mixamo skeleton)
└── README.md                 # This file
```

---

## Acknowledgements

- [cgltf](https://github.com/jkuhlmann/cgltf) – single‑header GLTF loader.
- [GLM](https://github.com/g-truc/glm) – OpenGL Mathematics.
- [Mixamo](https://www.mixamo.com/) – for the base 3D character model.
- The ESP‑NOW and MPU6050 libraries from the Arduino ecosystem.