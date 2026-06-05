#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "gltf_model.h"

enum class CameraView { FRONT, BACK, SIDE };

class Renderer {
public:
    void initialize();
    void render(const glm::quat& leftForearmQ,  const glm::quat& rightForearmQ,
                const glm::quat& leftUpperArmQ, const glm::quat& rightUpperArmQ);
    void setCameraView(CameraView view) { cameraView = view; }
    void cycleCameraView();

private:
    CameraView cameraView = CameraView::FRONT;

    void setupLighting();
    void drawWorldAxes();
    void drawTrackingAxesHud(const glm::quat& chestQ,
                             const glm::quat& upperArmQ,
                             const glm::quat& forearmQ,
                             const glm::quat& handQ);
    void drawHudAxisWidget(float cx, float cy,
                           const glm::quat& q,
                           float scale);

    // Body parts
    void drawBody();
    void drawHead(glm::vec3 pos);
    void drawNeck(glm::vec3 bottom, glm::vec3 top);
    void drawTorso(glm::vec3 shoulderCenter, glm::vec3 hipCenter);
    void drawPelvis(glm::vec3 center);
    void drawLeg(glm::vec3 hipJoint, bool mirrorSide);

    // Arm
    void drawArm(glm::vec3 shoulderPos,
                 const glm::quat& upperArmQ,
                 const glm::quat& forearmQ,
                 bool isRight);

    // Primitives
    void drawCylinder(glm::vec3 start, glm::vec3 end,
                      float radius, int segments,
                      float r, float g, float b);
    void drawSphere(glm::vec3 pos, float radius,
                    int segments, float r, float g, float b);
    void drawEllipsoid(glm::vec3 pos,
                       glm::vec3 right, glm::vec3 up, glm::vec3 fwd,
                       float halfW, float halfH, float halfD,
                       int segments, float r, float g, float b);
    void drawTaperedCylinder(glm::vec3 start, glm::vec3 end,
                             float startRadius, float endRadius,
                             int segments, float r, float g, float b);
    void drawFinger(glm::vec3 base,
                    glm::vec3 dir,
                    float totalLength,
                    float thickness,
                    float r,
                    float g,
                    float b);
    void drawHand(glm::vec3 wrist,
                  glm::vec3 forearmDir,
                  glm::vec3 handRight,
                  glm::vec3 handUp,
                  bool isRight,
                  float r, float g, float b);
    void drawBox(glm::vec3 center,
                 glm::vec3 right, glm::vec3 up, glm::vec3 fwd,
                 float halfW, float halfH, float halfD,
                 float r, float g, float b);

    // Body anchor points (computed in drawBody, used by drawArm)
    glm::vec3 leftShoulderPos  = glm::vec3(-1.4f,  5.5f, 0.0f);
    glm::vec3 rightShoulderPos = glm::vec3( 1.4f,  5.5f, 0.0f);

    // Skin / clothing colours
    static constexpr float kSkinR = 0.88f, kSkinG = 0.72f, kSkinB = 0.58f;
    static constexpr float kShirtR = 0.25f, kShirtG = 0.35f, kShirtB = 0.55f;
    static constexpr float kPantsR = 0.20f, kPantsG = 0.20f, kPantsB = 0.28f;
    static constexpr float kShoeR  = 0.15f, kShoeG  = 0.12f, kShoeB  = 0.10f;

    const int WINDOW_WIDTH  = 1400;
    const int WINDOW_HEIGHT = 900;

    GltfModel humanModel;
};
