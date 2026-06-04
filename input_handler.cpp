#include "input_handler.h"

InputHandler::InputHandler(SensorManager& sensorManager)
    : sensorManager(sensorManager)
{
}

void InputHandler::handleKey(int key)
{
    if      (key == GLFW_KEY_C) { sensorManager.calibrateHips();  }
    else if (key == GLFW_KEY_V) { sensorManager.calibrateChest(); }
    else if (key == GLFW_KEY_B) { sensorManager.calibrateLUA();   }
    else if (key == GLFW_KEY_N) { sensorManager.calibrateLFA();   }
    else if (key == GLFW_KEY_M) { sensorManager.toggleQuaternionConvention(); }
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
