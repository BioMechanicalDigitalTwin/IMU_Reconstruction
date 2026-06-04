#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class Renderer {
public:
    void initialize();
    void render(const glm::quat& correctedQ1, const glm::quat& correctedQ2,
                const glm::quat& correctedLUA, const glm::quat& correctedLFA);

private:
    void setupLighting();
    void drawWorldAxes();
    void drawArm(glm::vec3 worldOffset, glm::quat forearmQ,
             float skinR, float skinG, float skinB, bool mirrorThumb = false,
             glm::quat upperArmQ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
             bool hasUpperArmQ = false);
    void drawCylinder(glm::vec3 start, glm::vec3 end, 
                     float radius, int segments, float r, float g, float b);
    void drawSphere(glm::vec3 pos, float radius, 
                   int segments, float r, float g, float b);
    void drawFinger(glm::vec3 base, glm::vec3 dir, 
                   float thickness, float r, float g, float b);
    void drawInfinityStone(glm::vec3 position, float size, 
                          float r, float g, float b);
    void drawGauntletLines(glm::vec3 palmCenter, 
                          glm::vec3 handDir, 
                          glm::vec3 handRight, 
                          glm::vec3 handUp,
                          float halfPalmWidth, float halfPalmThickness, 
                          float halfPalmLength);
    
    const int WINDOW_WIDTH = 1200;
    const int WINDOW_HEIGHT = 800;
};