## To compile the program

```bash
g++ -std=c++17 -o imu_visualizer \
    main.cpp \
    sensor_manager.cpp \
    udp_receiver.cpp \
    input_handler.cpp \
    renderer.cpp \
    -lglfw -lGL -lGLU -lpthread \
    -I/usr/include/glm
```

## To run the executable

```bash
./imu_visualizer
```

## Controls

```text
C: calibrate HIPS   (left lower arm/hand)
V: calibrate CHEST  (right lower arm/hand)
B: calibrate L_UA   (left upper arm)
N: calibrate L_FA   (right upper arm)
M: cycle quaternion mode 1/4 -> 2/4 -> 3/4 -> 4/4, then recalibrate with C/V/B/N
```

During calibration, hold each arm in the neutral down pose. The active
quaternion mode controls how the raw sensor orientation is converted into
relative body motion. If one edge placement works but another does not, press
M, recalibrate with C/V/B/N, and test again.
