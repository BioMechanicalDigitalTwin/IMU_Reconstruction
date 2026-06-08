#include "input_handler.h"

InputHandler::InputHandler(SensorManager& sensorManager)
    : sensorManager(sensorManager)
{
}

void InputHandler::handleKey(int key)
{
    if      (key == GLFW_KEY_C)     { sensorManager.calibrateLFA();   }
    else if (key == GLFW_KEY_V)     { sensorManager.calibrateRFA();   }
    else if (key == GLFW_KEY_B)     { sensorManager.calibrateLUA();   }
    else if (key == GLFW_KEY_N)     { sensorManager.calibrateRUA();   }
    else if (key == GLFW_KEY_M)     { sensorManager.toggleQuaternionConvention(); }
    else if (key == GLFW_KEY_Z)     { sensorManager.calibrateLTH(); }
    else if (key == GLFW_KEY_X)     { sensorManager.calibrateLSH(); }
    else if (key == GLFW_KEY_G)     { sensorManager.calibrateRTH(); }
    else if (key == GLFW_KEY_H)     { sensorManager.calibrateRSH(); }
    else if (key == GLFW_KEY_SPACE) {
        sensorManager.calibrateLFA();
        sensorManager.calibrateRFA();
        sensorManager.calibrateLUA();
        sensorManager.calibrateRUA();
        sensorManager.calibrateLTH();
        sensorManager.calibrateLSH();
        sensorManager.calibrateRTH();
        sensorManager.calibrateRSH();
    }
}

void keyCallbackDispatcher(GLFWwindow* window, int key, int scancode, 
                          int action, int mods)
{
    if (action != GLFW_PRESS) return;
    
    InputHandler* handler = static_cast<InputHandler*>(
        glfwGetWindowUserPointer(window));
    
    if (handler) {
        handler->handleKey(key);
    }
}