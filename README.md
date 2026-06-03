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