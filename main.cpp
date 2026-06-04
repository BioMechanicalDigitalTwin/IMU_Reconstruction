#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "udp_receiver.h"
#include "sensor_manager.h"
#include "renderer.h"
#include "input_handler.h"

static GLFWwindow* g_window = nullptr;

int main()
{
    SensorManager sensorManager;
    std::thread receiver(udpReceiver, std::ref(sensorManager));

    if(!glfwInit())
        return -1;

    GLFWwindow* window = glfwCreateWindow(1200, 800, 
        "Quad IMU Visualizer - HIPS, CHEST, L_UA, L_FA", nullptr, nullptr);

    if(!window)
        return -1;

    g_window = window;
    glfwMakeContextCurrent(window);
    
    InputHandler inputHandler(sensorManager);
    glfwSetKeyCallback(window, keyCallbackDispatcher);
    glfwSetWindowUserPointer(window, &inputHandler);

    Renderer renderer;
    renderer.initialize();

    while(!glfwWindowShouldClose(window))
    {
        glm::quat correctedQ1  = sensorManager.getCorrectedHipsQuat();
        glm::quat correctedQ2  = sensorManager.getCorrectedChestQuat();
        glm::quat correctedLUA = sensorManager.getCorrectedLUAQuat();
        glm::quat correctedLFA = sensorManager.getCorrectedLFAQuat();

        renderer.render(correctedQ1, correctedQ2, correctedLUA, correctedLFA);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    receiver.detach();
    glfwTerminate();
    return 0;
}