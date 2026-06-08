#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>

// ─────────────────────────────────────────────
//  INIT / LIGHTING
// ─────────────────────────────────────────────

void Renderer::initialize()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setupLighting();
    humanModel.load("human.glb");
}

void Renderer::setupLighting()
{
    GLfloat amb[]  = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat diff[] = {0.90f, 0.88f, 0.84f, 1.0f};
    GLfloat spec[] = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat pos0[] = {8.0f, 18.0f, 10.0f, 1.0f};
    GLfloat pos1[] = {-5.0f, 10.0f, -6.0f, 1.0f};
    GLfloat fill[] = {0.25f, 0.25f, 0.28f, 1.0f};
    GLfloat zero[] = {0.0f,  0.0f,  0.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_POSITION, pos0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glLightfv(GL_LIGHT1, GL_POSITION, pos1);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  zero);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  fill);
    glLightfv(GL_LIGHT1, GL_SPECULAR, zero);
}

void Renderer::cycleCameraView()
{
    switch (cameraView) {
        case CameraView::FRONT: cameraView = CameraView::BACK;  break;
        case CameraView::BACK:  cameraView = CameraView::SIDE;  break;
        case CameraView::SIDE:  cameraView = CameraView::FRONT; break;
    }
}

// ─────────────────────────────────────────────
//  MAIN RENDER
// ─────────────────────────────────────────────

void Renderer::render(const glm::quat& leftForearmQ,  const glm::quat& rightForearmQ,
                      const glm::quat& leftUpperArmQ, const glm::quat& rightUpperArmQ,
                      const glm::quat& leftThighQ,    const glm::quat& rightThighQ,
                      const glm::quat& leftShinQ,     const glm::quat& rightShinQ)
{
    glClearColor(0.07f, 0.07f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT;
    glFrustum(-aspect * 0.5f, aspect * 0.5f, -0.5f, 0.5f, 1.5f, 200.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Body: feet at y=0, head top ~y=17.5
    // Camera looks at body centre ~y=9
    switch (cameraView) {
        case CameraView::FRONT:
            glTranslatef(0.0f, -9.0f, -38.0f);
            break;
        case CameraView::BACK:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
            break;
        case CameraView::SIDE:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
            break;
    }

    setupLighting();
    if (humanModel.isLoaded()) {
        humanModel.draw(leftForearmQ, rightForearmQ, leftUpperArmQ, rightUpperArmQ);
    } else {
        drawBody();
        drawArm(leftShoulderPos,  leftUpperArmQ,  leftForearmQ,  false);
        drawArm(rightShoulderPos, rightUpperArmQ, rightForearmQ, true);
        drawLegSensor(leftHipPos,  leftThighQ,  leftShinQ,  false);
        drawLegSensor(rightHipPos, rightThighQ, rightShinQ, true);
    }
    drawWorldAxes();
    drawTrackingAxesHud(rightForearmQ, leftUpperArmQ, leftForearmQ, leftForearmQ);
}

// ─────────────────────────────────────────────
//  BODY  (real human proportions, ~7.5 heads tall)
//  Head height = 2.2 units
//  Total height = ~17.5 units
// ─────────────────────────────────────────────

void Renderer::drawBody()
{
    const glm::vec3 X(1,0,0), Y(0,1,0), Z(0,0,1);
    const float hipY      = 8.60f;
    const float abdomenY  = 10.35f;
    const float ribcageY  = 12.00f;
    const float shoulderY = 13.70f;
    const float neckBotY  = 13.55f;
    const float neckTopY  = 14.70f;
    const float headY     = 15.72f;
    const float kneeY     = 5.20f;
    const float ankleY    = 1.20f;

    leftShoulderPos  = glm::vec3(-2.00f, shoulderY, 0.02f);
    rightShoulderPos = glm::vec3( 2.00f, shoulderY, 0.02f);

    // Low-poly engineering mannequin volumes: clear landmarks over character detail.
    drawEllipsoid(glm::vec3(0, hipY, -0.02f), X, Y, Z,
                  1.30f, 0.72f, 0.50f, 18,
                  kSkinR*0.90f, kSkinG*0.82f, kSkinB*0.76f);
    drawEllipsoid(glm::vec3(0, abdomenY, 0.00f), X, Y, Z,
                  1.14f, 1.00f, 0.46f, 18,
                  kSkinR*0.96f, kSkinG*0.88f, kSkinB*0.81f);
    drawEllipsoid(glm::vec3(0, ribcageY, 0.03f), X, Y, Z,
                  1.50f, 1.32f, 0.50f, 20,
                  kSkinR, kSkinG*0.92f, kSkinB*0.84f);

    drawTaperedCylinder(glm::vec3(0.0f, 10.90f, 0.48f), glm::vec3(0.0f, 13.35f, 0.48f),
                        0.040f, 0.030f, 8,
                        kSkinR*0.75f, kSkinG*0.67f, kSkinB*0.62f);
    drawTaperedCylinder(glm::vec3(-0.20f, 13.32f, 0.42f), glm::vec3(-1.55f, 13.68f, 0.30f),
                        0.10f, 0.08f, 8,
                        kSkinR*0.72f, kSkinG*0.64f, kSkinB*0.60f);
    drawTaperedCylinder(glm::vec3( 0.20f, 13.32f, 0.42f), glm::vec3( 1.55f, 13.68f, 0.30f),
                        0.10f, 0.08f, 8,
                        kSkinR*0.72f, kSkinG*0.64f, kSkinB*0.60f);

    drawTaperedCylinder(glm::vec3(0, neckBotY, 0.02f), glm::vec3(0, neckTopY, 0.02f),
                        0.26f, 0.22f, 16,
                        kSkinR*0.94f, kSkinG*0.86f, kSkinB*0.80f);
    drawHead(glm::vec3(0, headY, 0.02f));

    drawEllipsoid(leftShoulderPos, X, Y, Z,
                  0.38f, 0.33f, 0.32f, 16,
                  kSkinR*0.96f, kSkinG*0.88f, kSkinB*0.82f);
    drawEllipsoid(rightShoulderPos, X, Y, Z,
                  0.38f, 0.33f, 0.32f, 16,
                  kSkinR*0.96f, kSkinG*0.88f, kSkinB*0.82f);

    const float hipX = 0.72f;
    for (int side = -1; side <= 1; side += 2) {
        float s = (float)side;
        glm::vec3 hip   = glm::vec3(s * hipX, 8.55f, 0.02f);
        glm::vec3 knee  = glm::vec3(s * (hipX + 0.06f), kneeY, 0.07f);
        glm::vec3 calf  = glm::vec3(s * (hipX + 0.03f), 3.20f, -0.06f);
        glm::vec3 ankle = glm::vec3(s * (hipX - 0.02f), ankleY, 0.00f);
        glm::vec3 foot  = glm::vec3(s * (hipX - 0.02f), 0.45f, 0.65f);

        drawEllipsoid(hip, X, Y, Z,
                      0.40f, 0.34f, 0.34f, 14,
                      kSkinR*0.88f, kSkinG*0.80f, kSkinB*0.74f);
        drawTaperedCylinder(hip, knee, 0.43f, 0.28f, 18,
                            kSkinR*0.95f, kSkinG*0.87f, kSkinB*0.80f);
        drawEllipsoid(knee, X, Y, Z,
                      0.30f, 0.24f, 0.26f, 14,
                      kSkinR*0.84f, kSkinG*0.76f, kSkinB*0.70f);
        drawTaperedCylinder(knee, calf, 0.28f, 0.35f, 18,
                            kSkinR*0.93f, kSkinG*0.85f, kSkinB*0.78f);
        drawTaperedCylinder(calf, ankle, 0.35f, 0.18f, 18,
                            kSkinR*0.96f, kSkinG*0.88f, kSkinB*0.81f);
        drawEllipsoid(ankle, X, Y, Z,
                      0.20f, 0.16f, 0.18f, 12,
                      kSkinR*0.83f, kSkinG*0.75f, kSkinB*0.69f);
        drawEllipsoid(foot, X, Y, Z,
                      0.36f, 0.24f, 1.05f, 16,
                      kSkinR*0.90f, kSkinG*0.82f, kSkinB*0.75f);
        drawEllipsoid(foot + glm::vec3(0, -0.02f, 0.78f), X, Y, Z,
                      0.40f, 0.12f, 0.34f, 14,
                      kSkinR*0.88f, kSkinG*0.80f, kSkinB*0.73f);
    }
}

// ─────────────────────────────────────────────
//  HEAD
// ─────────────────────────────────────────────

void Renderer::drawHead(glm::vec3 pos)
{
    const glm::vec3 X(1,0,0), Y(0,1,0), Z(0,0,1);

    drawEllipsoid(pos, X, Y, Z,
                  0.76f, 1.10f, 0.56f, 22,
                  kSkinR, kSkinG*0.94f, kSkinB*0.87f);
    drawEllipsoid(pos + glm::vec3(0.0f, -0.54f, 0.18f), X, Y, Z,
                  0.50f, 0.34f, 0.36f, 16,
                  kSkinR*0.94f, kSkinG*0.86f, kSkinB*0.80f);
    drawEllipsoid(pos + glm::vec3(0.0f, -0.88f, 0.30f), X, Y, Z,
                  0.26f, 0.13f, 0.19f, 12,
                  kSkinR*0.88f, kSkinG*0.80f, kSkinB*0.74f);

    // Low-contrast anatomical face details keep the head realistic instead of cartoon-like.
    drawEllipsoid(pos + glm::vec3(0.0f, 0.22f, 0.57f), X, Y, Z,
                  0.56f, 0.10f, 0.05f, 14,
                  kSkinR*0.78f, kSkinG*0.70f, kSkinB*0.66f);
    drawTaperedCylinder(pos + glm::vec3(0.0f, 0.16f, 0.64f), pos + glm::vec3(0.0f, -0.28f, 0.82f),
                        0.10f, 0.07f, 12,
                        kSkinR*0.90f, kSkinG*0.80f, kSkinB*0.74f);
    drawEllipsoid(pos + glm::vec3(0.0f, -0.31f, 0.86f), X, Y, Z,
                  0.13f, 0.09f, 0.10f, 12,
                  kSkinR*0.82f, kSkinG*0.72f, kSkinB*0.67f);
    drawEllipsoid(pos + glm::vec3(-0.24f, 0.08f, 0.64f), X, Y, Z,
                  0.09f, 0.035f, 0.025f, 10,
                  0.08f, 0.06f, 0.05f);
    drawEllipsoid(pos + glm::vec3( 0.24f, 0.08f, 0.64f), X, Y, Z,
                  0.09f, 0.035f, 0.025f, 10,
                  0.08f, 0.06f, 0.05f);
    drawEllipsoid(pos + glm::vec3(0.0f, -0.54f, 0.68f), X, Y, Z,
                  0.24f, 0.035f, 0.018f, 10,
                  kSkinR*0.58f, kSkinG*0.46f, kSkinB*0.44f);

    drawEllipsoid(pos + glm::vec3(-0.76f, -0.02f, 0.02f), X, Y, Z,
                  0.13f, 0.28f, 0.10f, 14,
                  kSkinR*0.86f, kSkinG*0.78f, kSkinB*0.72f);
    drawEllipsoid(pos + glm::vec3( 0.76f, -0.02f, 0.02f), X, Y, Z,
                  0.13f, 0.28f, 0.10f, 14,
                  kSkinR*0.86f, kSkinG*0.78f, kSkinB*0.72f);
}

// ─────────────────────────────────────────────
//  SENSOR-DRIVEN ARM
// ─────────────────────────────────────────────

void Renderer::drawArm(glm::vec3 shoulderPos,
                        const glm::quat& upperArmQ,
                        const glm::quat& forearmQ,
                        bool isRight)
{
    const glm::vec3 down(0.0f, -1.0f, 0.0f);
    const float upperLen = 2.70f;   // shoulder → elbow
    const float foreLen  = 2.50f;   // elbow → wrist

    glm::vec3 upperDir = glm::normalize(upperArmQ * down);
    glm::vec3 elbow    = shoulderPos + upperDir * upperLen;

    glm::vec3 foreDir  = glm::normalize(forearmQ * down);
    glm::vec3 wrist    = elbow + foreDir * foreLen;

    // Anatomical arm volumes; the kinematic chain above is intentionally unchanged.
    drawTaperedCylinder(shoulderPos, elbow, 0.30f, 0.23f, 18,
                        kSkinR*0.95f, kSkinG*0.87f, kSkinB*0.80f);
    drawEllipsoid(elbow, glm::vec3(1,0,0), glm::vec3(0,1,0), glm::vec3(0,0,1),
                  0.25f, 0.21f, 0.22f, 12,
                  kSkinR*0.82f, kSkinG*0.74f, kSkinB*0.68f);

    drawTaperedCylinder(elbow, wrist, 0.24f, 0.15f, 18,
                        kSkinR, kSkinG*0.92f, kSkinB*0.84f);
    drawEllipsoid(wrist, glm::vec3(1,0,0), glm::vec3(0,1,0), glm::vec3(0,0,1),
                  0.16f, 0.14f, 0.15f, 12,
                  kSkinR*0.84f, kSkinG*0.76f, kSkinB*0.70f);

    // Hand
    glm::vec3 handRight = glm::normalize(forearmQ * glm::vec3(isRight ? 1.0f : -1.0f, 0.0f, 0.0f));
    glm::vec3 handUp    = glm::normalize(forearmQ * glm::vec3(0.0f, 0.0f, 1.0f));
    drawHand(wrist, foreDir, handRight, handUp, isRight,
             kSkinR, kSkinG, kSkinB);

    const bool showJointAxes = false;
    if (showJointAxes) {
        glDisable(GL_LIGHTING);
        glm::vec3 ax = forearmQ * glm::vec3(1,0,0);
        glm::vec3 ay = forearmQ * glm::vec3(0,1,0);
        glm::vec3 az = forearmQ * glm::vec3(0,0,1);
        glBegin(GL_LINES);
        glColor3f(1,0,0); glVertex3fv(&elbow.x); glm::vec3 t=elbow+ax*1.5f; glVertex3fv(&t.x);
        glColor3f(0,1,0); glVertex3fv(&elbow.x);              t=elbow+ay*1.5f; glVertex3fv(&t.x);
        glColor3f(0,0,1); glVertex3fv(&elbow.x);              t=elbow+az*1.5f; glVertex3fv(&t.x);
        glEnd();
        glEnable(GL_LIGHTING);
    }
}

// ─────────────────────────────────────────────
//  HAND  (proportionate: ~0.75× forearm length)
// ─────────────────────────────────────────────

void Renderer::drawHand(glm::vec3 wrist,
                         glm::vec3 foreDir,
                         glm::vec3 handRight,
                         glm::vec3 handUp,
                         bool isRight,
                         float r, float g, float b)
{
    // Palm dimensions
    const float palmLen  = 0.72f;   // wrist → knuckles
    const float halfW    = 0.26f;   // half palm width
    const float halfH    = 0.08f;   // half palm thickness

    glm::vec3 palmCenter = wrist + foreDir * palmLen * 0.5f;
    glm::vec3 knuckleLine = wrist + foreDir * palmLen;

    drawEllipsoid(palmCenter, handRight, handUp, foreDir,
                  halfW, halfH, palmLen * 0.5f, 14,
                  r*0.96f, g*0.86f, b*0.72f);

    // Knuckle bumps
    for (int i = -2; i <= 2; i++) {
        glm::vec3 k = knuckleLine + handRight * (i * halfW * 0.34f);
        drawSphere(k, 0.045f, 8, r*0.84f, g*0.76f, b*0.65f);
    }

    // Four fingers (index → pinky)
    float spreads[4]    = { 0.18f,  0.06f, -0.06f, -0.18f};
    float lengths[4]    = { 0.62f,  0.70f,  0.64f,  0.50f};
    float thicks[4]     = { 0.045f, 0.052f, 0.046f, 0.038f};

    for (int i = 0; i < 4; i++) {
        float side = isRight ? 1.0f : -1.0f;
        glm::vec3 base = knuckleLine + handRight * spreads[i] * side;
        drawFinger(base, foreDir, lengths[i], thicks[i],
                   r*0.94f, g*0.83f, b*0.68f);
    }

    // Thumb
    float side = isRight ? 1.0f : -1.0f;
    glm::vec3 thumbBase = wrist + handRight * halfW * 0.80f * side
                                + foreDir   * 0.18f
                                - handUp    * 0.05f;
    glm::vec3 thumbDir  = glm::normalize(foreDir * 0.34f
                                       + handRight * 0.76f * side
                                       + handUp    * 0.18f);
    drawFinger(thumbBase, thumbDir, 0.48f, 0.060f,
               r, g*0.83f, b*0.68f);
}

void Renderer::drawLegSensor(glm::vec3 hipPos,
                              const glm::quat& thighQ,
                              const glm::quat& shinQ,
                              bool isRight)
{
    const glm::vec3 down(0.0f, -1.0f, 0.0f);
    const float thighLen = 4.20f;
    const float shinLen  = 3.80f;

    glm::vec3 thighDir = glm::normalize(thighQ * down);
    glm::vec3 knee     = hipPos + thighDir * thighLen;

    glm::vec3 shinDir  = glm::normalize(shinQ * down);
    glm::vec3 ankle    = knee + shinDir * shinLen;

    drawSphere  (hipPos, 0.54f, 18, kPantsR*1.3f, kPantsG*1.3f, kPantsB*1.4f);
    drawCylinder(hipPos, knee,  0.46f, 18, kPantsR, kPantsG, kPantsB);
    drawSphere  (knee,   0.40f, 16, kPantsR*1.2f, kPantsG*1.2f, kPantsB*1.3f);
    drawCylinder(knee,   ankle, 0.36f, 16, kPantsR*1.05f, kPantsG*1.05f, kPantsB*1.05f);
    drawSphere  (ankle,  0.34f, 14, kShoeR*1.4f,  kShoeG*1.3f,  kShoeB*1.3f);

    // Foot
    glm::vec3 footFwd = glm::normalize(shinQ * glm::vec3(0,0,1));
    glm::vec3 footRight = glm::normalize(shinQ * glm::vec3(isRight ? 1.0f : -1.0f, 0,0));
    drawBox(ankle + shinDir*0.20f + footFwd*0.32f,
            footRight, glm::vec3(0,1,0), footFwd,
            0.30f, 0.14f, 0.58f,
            kShoeR, kShoeG, kShoeB);
}

// ─────────────────────────────────────────────
//  WORLD AXES
// ─────────────────────────────────────────────

void Renderer::drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(3,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,3,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,3);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::drawTrackingAxesHud(const glm::quat& chestQ,
                                   const glm::quat& upperArmQ,
                                   const glm::quat& forearmQ,
                                   const glm::quat& handQ)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, WINDOW_WIDTH, 0.0, WINDOW_HEIGHT, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    const float x = (float)WINDOW_WIDTH - 58.0f;
    const float top = (float)WINDOW_HEIGHT - 42.0f;
    const float gap = 44.0f;
    const float scale = 18.0f;

    drawHudAxisWidget(x, top,             chestQ,    scale);
    drawHudAxisWidget(x, top - gap,       upperArmQ, scale);
    drawHudAxisWidget(x, top - gap * 2.0f, forearmQ, scale);
    drawHudAxisWidget(x, top - gap * 3.0f, handQ,    scale);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void Renderer::drawHudAxisWidget(float cx, float cy,
                                 const glm::quat& q,
                                 float scale)
{
    glm::mat4 viewRot(1.0f);
    switch (cameraView) {
        case CameraView::FRONT:
            break;
        case CameraView::BACK:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0,1,0));
            break;
        case CameraView::SIDE:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0));
            break;
    }

    glm::vec3 axes[3] = {
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(1,0,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,1,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,0,1), 0.0f))
    };

    glColor3f(0.22f, 0.22f, 0.26f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(cx - 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy + 20.0f);
    glVertex2f(cx - 24.0f, cy + 20.0f);
    glEnd();

    glPointSize(4.0f);
    glColor3f(0.85f, 0.85f, 0.88f);
    glBegin(GL_POINTS);
    glVertex2f(cx, cy);
    glEnd();

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.12f, 0.10f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[0].x * scale, cy + axes[0].y * scale);

    glColor3f(0.20f, 1.0f, 0.20f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[1].x * scale, cy + axes[1].y * scale);

    glColor3f(0.20f, 0.45f, 1.0f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[2].x * scale, cy + axes[2].y * scale);
    glEnd();
    glLineWidth(1.0f);
    glPointSize(1.0f);
}

// ─────────────────────────────────────────────
//  PRIMITIVES
// ─────────────────────────────────────────────

void Renderer::drawCylinder(glm::vec3 start, glm::vec3 end,
                              float radius, int segments,
                              float r, float g, float b)
{
    glm::vec3 dir = end - start;
    float len = glm::length(dir);
    if (len < 1e-6f) return;
    dir = glm::normalize(dir);

    glm::vec3 p1 = (fabs(dir.x) < 0.9f)
        ? glm::normalize(glm::cross(dir, glm::vec3(1,0,0)))
        : glm::normalize(glm::cross(dir, glm::vec3(0,1,0)));
    glm::vec3 p2 = glm::cross(dir, p1);

    std::vector<glm::vec3> c1, c2;
    for (int i = 0; i < segments; i++) {
        float a = 2.0f*(float)M_PI*i/segments;
        glm::vec3 off = p1*cosf(a)*radius + p2*sinf(a)*radius;
        c1.push_back(start+off);
        c2.push_back(end+off);
    }

    glColor3f(r*0.82f, g*0.82f, b*0.82f);
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; i++) {
        int idx = i%segments;
        glm::vec3 n = glm::normalize(c1[idx]-start);
        glNormal3fv(&n.x);
        glVertex3fv(&c1[idx].x);
        glVertex3fv(&c2[idx].x);
    }
    glEnd();

    glColor3f(r, g, b);
    glm::vec3 n = -dir;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3fv(&n.x); glVertex3fv(&start.x);
    for (int i = 0; i <= segments; i++) glVertex3fv(&c1[i%segments].x);
    glEnd();
    n = dir;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3fv(&n.x); glVertex3fv(&end.x);
    for (int i = segments; i >= 0; i--) glVertex3fv(&c2[i%segments].x);
    glEnd();
}

void Renderer::drawSphere(glm::vec3 pos, float radius, int segments,
                           float r, float g, float b)
{
    glColor3f(r, g, b);
    for (int i = 0; i <= segments; i++) {
        float lat0 = (float)M_PI*(-0.5f+(float)(i-1)/segments);
        float lat1 = (float)M_PI*(-0.5f+(float)i    /segments);
        float z0=sinf(lat0), zr0=cosf(lat0);
        float z1=sinf(lat1), zr1=cosf(lat1);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; j++) {
            float lng = 2.0f*(float)M_PI*(float)(j-1)/segments;
            float cx=cosf(lng), cy=sinf(lng);
            glm::vec3 n0(cx*zr0,cy*zr0,z0); glNormal3fv(&n0.x);
            glVertex3f(pos.x+cx*zr0*radius, pos.y+cy*zr0*radius, pos.z+z0*radius);
            glm::vec3 n1(cx*zr1,cy*zr1,z1); glNormal3fv(&n1.x);
            glVertex3f(pos.x+cx*zr1*radius, pos.y+cy*zr1*radius, pos.z+z1*radius);
        }
        glEnd();
    }
}

void Renderer::drawEllipsoid(glm::vec3 pos,
                              glm::vec3 right, glm::vec3 up, glm::vec3 fwd,
                              float halfW, float halfH, float halfD,
                              int segments, float r, float g, float b)
{
    right = glm::normalize(right);
    up    = glm::normalize(up);
    fwd   = glm::normalize(fwd);

    glColor3f(r, g, b);
    for (int i = 0; i <= segments; i++) {
        float lat0 = (float)M_PI * (-0.5f + (float)(i - 1) / segments);
        float lat1 = (float)M_PI * (-0.5f + (float)i / segments);
        float y0 = sinf(lat0), yr0 = cosf(lat0);
        float y1 = sinf(lat1), yr1 = cosf(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; j++) {
            float lng = 2.0f * (float)M_PI * (float)(j - 1) / segments;
            float x = cosf(lng);
            float z = sinf(lng);

            glm::vec3 n0 = glm::normalize(right * (x * yr0 / halfW) +
                                           up    * (y0      / halfH) +
                                           fwd   * (z * yr0 / halfD));
            glm::vec3 p0 = pos + right * (x * yr0 * halfW) +
                                 up    * (y0      * halfH) +
                                 fwd   * (z * yr0 * halfD);
            glNormal3fv(&n0.x);
            glVertex3fv(&p0.x);

            glm::vec3 n1 = glm::normalize(right * (x * yr1 / halfW) +
                                           up    * (y1      / halfH) +
                                           fwd   * (z * yr1 / halfD));
            glm::vec3 p1 = pos + right * (x * yr1 * halfW) +
                                 up    * (y1      * halfH) +
                                 fwd   * (z * yr1 * halfD);
            glNormal3fv(&n1.x);
            glVertex3fv(&p1.x);
        }
        glEnd();
    }
}

void Renderer::drawTaperedCylinder(glm::vec3 start, glm::vec3 end,
                                    float startRadius, float endRadius,
                                    int segments, float r, float g, float b)
{
    glm::vec3 dir = end - start;
    float len = glm::length(dir);
    if (len < 1e-6f) return;
    dir = glm::normalize(dir);

    glm::vec3 p1 = (fabs(dir.x) < 0.9f)
        ? glm::normalize(glm::cross(dir, glm::vec3(1,0,0)))
        : glm::normalize(glm::cross(dir, glm::vec3(0,1,0)));
    glm::vec3 p2 = glm::cross(dir, p1);

    std::vector<glm::vec3> c1, c2;
    for (int i = 0; i < segments; i++) {
        float a = 2.0f * (float)M_PI * i / segments;
        glm::vec3 radial = p1 * cosf(a) + p2 * sinf(a);
        c1.push_back(start + radial * startRadius);
        c2.push_back(end   + radial * endRadius);
    }

    glColor3f(r*0.88f, g*0.88f, b*0.88f);
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; i++) {
        int idx = i % segments;
        glm::vec3 n = glm::normalize((c1[idx] - start) + (c2[idx] - end));
        glNormal3fv(&n.x);
        glVertex3fv(&c1[idx].x);
        glVertex3fv(&c2[idx].x);
    }
    glEnd();

    glColor3f(r, g, b);
    glm::vec3 n = -dir;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3fv(&n.x); glVertex3fv(&start.x);
    for (int i = 0; i <= segments; i++) glVertex3fv(&c1[i%segments].x);
    glEnd();

    n = dir;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3fv(&n.x); glVertex3fv(&end.x);
    for (int i = segments; i >= 0; i--) glVertex3fv(&c2[i%segments].x);
    glEnd();
}

void Renderer::drawFinger(glm::vec3 base, glm::vec3 dir,
                           float totalLength, float thickness,
                           float r, float g, float b)
{
    // 3 phalanges: 45% / 32% / 23% of total length
    float segs[3] = {totalLength*0.45f, totalLength*0.32f, totalLength*0.23f};
    glm::vec3 start = base;
    for (int i = 0; i < 3; i++) {
        glm::vec3 end = start + dir * segs[i];
        drawCylinder(start, end, thickness, 10, r, g, b);
        drawSphere(end, thickness*1.18f, 8, r*0.86f, g*0.78f, b*0.68f);
        start = end;
        thickness *= 0.82f;
    }
}

void Renderer::drawBox(glm::vec3 center,
                        glm::vec3 right, glm::vec3 up, glm::vec3 fwd,
                        float halfW, float halfH, float halfD,
                        float r, float g, float b)
{
    glm::vec3 c[8];
    c[0] = center + right*halfW + up*halfH + fwd*halfD;
    c[1] = center + right*halfW + up*halfH - fwd*halfD;
    c[2] = center - right*halfW + up*halfH + fwd*halfD;
    c[3] = center - right*halfW + up*halfH - fwd*halfD;
    c[4] = center + right*halfW - up*halfH + fwd*halfD;
    c[5] = center + right*halfW - up*halfH - fwd*halfD;
    c[6] = center - right*halfW - up*halfH + fwd*halfD;
    c[7] = center - right*halfW - up*halfH - fwd*halfD;

    // Front (+fwd)
    glColor3f(r,g,b);
    glm::vec3 n=fwd; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[0].x); glVertex3fv(&c[4].x);
    glVertex3fv(&c[6].x); glVertex3fv(&c[2].x);
    glEnd();
    // Back (-fwd)
    glColor3f(r*0.84f,g*0.84f,b*0.84f);
    n=-fwd; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[1].x); glVertex3fv(&c[3].x);
    glVertex3fv(&c[7].x); glVertex3fv(&c[5].x);
    glEnd();
    // Top (+up)
    glColor3f(r*1.06f,g*1.06f,b*1.06f);
    n=up; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[0].x); glVertex3fv(&c[1].x);
    glVertex3fv(&c[3].x); glVertex3fv(&c[2].x);
    glEnd();
    // Bottom (-up)
    glColor3f(r*0.74f,g*0.74f,b*0.74f);
    n=-up; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[4].x); glVertex3fv(&c[6].x);
    glVertex3fv(&c[7].x); glVertex3fv(&c[5].x);
    glEnd();
    // Right (+right)
    glColor3f(r*0.91f,g*0.91f,b*0.91f);
    n=right; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[0].x); glVertex3fv(&c[2].x);
    glVertex3fv(&c[6].x); glVertex3fv(&c[4].x);
    glEnd();
    // Left (-right)
    glColor3f(r*0.91f,g*0.91f,b*0.91f);
    n=-right; glNormal3fv(&n.x);
    glBegin(GL_QUADS);
    glVertex3fv(&c[1].x); glVertex3fv(&c[5].x);
    glVertex3fv(&c[7].x); glVertex3fv(&c[3].x);
    glEnd();
}
