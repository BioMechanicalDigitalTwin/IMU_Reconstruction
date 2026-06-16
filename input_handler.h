#pragma once
#include <GLFW/glfw3.h>
#include "sensor_manager.h"
#include "csv_logger.h"

class InputHandler {
public:
    InputHandler(SensorManager& sensorManager, CsvLogger& csvLogger);
    void handleKey(int key);

private:
    SensorManager& sensorManager;
    CsvLogger&     csvLogger;
};

void keyCallbackDispatcher(GLFWwindow* window, int key, int scancode,
                           int action, int mods);