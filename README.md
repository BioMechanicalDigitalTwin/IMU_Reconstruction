## Compilation

```bash
g++ -std=c++17 -o imu_visualizer \
    main.cpp sensor_manager.cpp udp_receiver.cpp \
    input_handler.cpp renderer.cpp gltf_model.cpp \
    -lglfw -lGL -lGLU -lglut -lpthread \
    -I/usr/include/glm -Ithird_party
```

## Execution

```bash
./imu_visualizer
```

Place `human.glb` in the same directory. The program requires this GLB model (Mixamo‑style skeleton with skinning) – no fallback renderer is used.

## UDP Input Format

The application listens on UDP port 5005. Each packet is a comma‑separated line:

```
<label>,w,x,y,z
```

Supported labels: `L_FA`, `R_FA`, `L_UA`, `R_UA`, `L_TH`, `R_TH`, `L_SH`, `R_SH`, `HIPS`, `CHEST`

Example packet:
```
L_FA,0.707,-0.707,0.000,0.000
```

## Calibration & Controls

Hold the neutral pose (arms down, legs straight, torso upright), then press the calibration keys.  
The active quaternion mode determines how raw sensor data is converted to body motion.

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
| `M` | Cycle quaternion mode (1/4 → 2/4 → 3/4 → 4/4) |
| `Space` | Re‑calibrate **all sensors at once** |
| `1` | Camera: front view             |
| `2` | Camera: back view              |
| `3` | Camera: side view              |

After pressing `M`, you **must** recalibrate (e.g., with `C`/`V`/`B`/`N` or `Space`) for the new mode to take effect.

## Troubleshooting

- If the GLB fails to load, check that `human.glb` exists and is a valid glTF binary file.
- If body parts move incorrectly, try a different quaternion mode (`M`) and recalibrate.
- Ensure sensor data is sent as unit quaternions (normalised, w first).

---

## Quaternion Modes

The sensor delivers a raw orientation quaternion. The program computes a motion delta between the current reading and the calibration reference, then combines it with a neutral pose. The 4 modes change the order of multiplication and whether we use conjugate quaternions – this effectively swaps sensor axes or changes the rotation’s handedness.

- **Mode 1 (default)**: `delta = current * inverse(reference)`  
  Standard relative rotation (sensor‑to‑world). Works when the sensor axes align with the body axes.

- **Mode 2**: `delta = inverse(reference) * current`  
  Reverses multiplication order. Useful if the sensor is mounted “backwards” relative to the body.

- **Mode 3**: `delta = conjugate(current) * inverse(conjugate(reference))`  
  Conjugates (inverts) both quaternions before computing delta. Flips the sign of the vector part, effectively mirroring the rotation.

- **Mode 4**: `delta = inverse(conjugate(reference)) * conjugate(current)`  
  Combination of conjugation and order reversal. Provides the final axis remapping.

If the body part moves in the wrong direction (e.g., arm bends the wrong way), press **M** to cycle modes, then recalibrate. One mode will produce natural motion.