#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>

void Renderer::initialize()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    setupLighting();
}

void Renderer::setupLighting()
{
    GLfloat light_pos[] = { 5.0f, 10.0f, 5.0f, 1.0f };
    GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat light_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat light_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
}

void Renderer::render(const glm::quat& correctedQ1, const glm::quat& correctedQ2,
                      const glm::quat& correctedLUA, const glm::quat& correctedLFA)
{
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    float aspect = (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT;
    glFrustum(-aspect, aspect, -1.0f, 1.0f, 2.0f, 100.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glTranslatef(0.0f, 0.0f, -20.0f);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    
    drawWorldAxes();
    
    // worldOffset is now the SHOULDER (fixed hinge point)
    drawArm(glm::vec3(-7.0f, 0.0f, 0.0f), correctedQ1,
            0.9f, 0.8f, 0.7f, false, correctedLUA, true);
    drawArm(glm::vec3( 7.0f, 0.0f, 0.0f), correctedQ2,
            0.7f, 0.8f, 0.9f, true, correctedLFA, true);
}

void Renderer::drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(3.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 3.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 3.0f);
    glEnd();
    glEnable(GL_LIGHTING);
}

void Renderer::drawCylinder(glm::vec3 start, glm::vec3 end, float radius, int segments, float r, float g, float b)
{
    glm::vec3 dir = end - start;
    float length = glm::length(dir);
    dir = glm::normalize(dir);
    
    glm::vec3 perp1, perp2;
    if (fabs(dir.x) < 0.9f) {
        perp1 = glm::normalize(glm::cross(dir, glm::vec3(1.0f, 0.0f, 0.0f)));
    } else {
        perp1 = glm::normalize(glm::cross(dir, glm::vec3(0.0f, 1.0f, 0.0f)));
    }
    perp2 = glm::cross(dir, perp1);
    
    std::vector<glm::vec3> circle1, circle2;
    for (int i = 0; i < segments; i++) {
        float angle = 2.0f * (float)M_PI * i / segments;
        float cosAngle = (float)cos(angle);
        float sinAngle = (float)sin(angle);
        glm::vec3 offset = perp1 * cosAngle * radius + perp2 * sinAngle * radius;
        circle1.push_back(start + offset);
        circle2.push_back(end + offset);
    }
    
    glColor3f(r * 0.8f, g * 0.8f, b * 0.8f);
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; i++) {
        int idx = i % segments;
        glm::vec3 normal = glm::normalize(circle1[idx] - start);
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(circle1[idx].x, circle1[idx].y, circle1[idx].z);
        glVertex3f(circle2[idx].x, circle2[idx].y, circle2[idx].z);
    }
    glEnd();
    
    glColor3f(r, g, b);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-dir.x, -dir.y, -dir.z);
    glVertex3f(start.x, start.y, start.z);
    for (int i = 0; i <= segments; i++) {
        int idx = i % segments;
        glVertex3f(circle1[idx].x, circle1[idx].y, circle1[idx].z);
    }
    glEnd();
    
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(dir.x, dir.y, dir.z);
    glVertex3f(end.x, end.y, end.z);
    for (int i = segments; i >= 0; i--) {
        int idx = i % segments;
        glVertex3f(circle2[idx].x, circle2[idx].y, circle2[idx].z);
    }
    glEnd();
}

void Renderer::drawSphere(glm::vec3 pos, float radius, int segments, float r, float g, float b)
{
    for (int i = 0; i <= segments; i++) {
        float lat0 = (float)M_PI * (-0.5f + (float)(i - 1) / segments);
        float z0  = (float)sin(lat0);
        float zr0 = (float)cos(lat0);
        
        float lat1 = (float)M_PI * (-0.5f + (float)i / segments);
        float z1 = (float)sin(lat1);
        float zr1 = (float)cos(lat1);
        
        glColor3f(r, g, b);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; j++) {
            float lng = 2.0f * (float)M_PI * (float)(j - 1) / segments;
            float x = (float)cos(lng);
            float y = (float)sin(lng);
            
            glm::vec3 normal = glm::normalize(glm::vec3(x * zr0, y * zr0, z0));
            glNormal3f(normal.x, normal.y, normal.z);
            glVertex3f(pos.x + x * zr0 * radius, pos.y + y * zr0 * radius, pos.z + z0 * radius);
            
            normal = glm::normalize(glm::vec3(x * zr1, y * zr1, z1));
            glNormal3f(normal.x, normal.y, normal.z);
            glVertex3f(pos.x + x * zr1 * radius, pos.y + y * zr1 * radius, pos.z + z1 * radius);
        }
        glEnd();
    }
}

void Renderer::drawFinger(glm::vec3 base, glm::vec3 dir, float thickness, float r, float g, float b)
{
    int segments = 3;
    float segLength = 0.4f;
    float segThickness = thickness;
    
    glm::vec3 start = base;
    for (int i = 0; i < segments; i++) {
        glm::vec3 end = start + dir * segLength;
        drawCylinder(start, end, segThickness, 12, r, g, b);
        drawSphere(end, segThickness * 1.15f, 12, r * 0.9f, g * 0.9f, b * 0.9f);
        start = end;
        segThickness *= 0.8f;
    }
    
    drawSphere(start, segThickness * 1.2f, 12, r * 0.95f, g * 0.8f, b * 0.7f);
}

void Renderer::drawInfinityStone(glm::vec3 position, float size, float r, float g, float b)
{
    glDisable(GL_LIGHTING);
    
    for (int i = 0; i < 3; i++) {
        float glowSize = size * (1.0f + i * 0.2f);
        glColor4f(r, g, b, 0.3f - i * 0.1f);
        drawSphere(position, glowSize, 12, r * (1.0f - i * 0.1f), g * (1.0f - i * 0.1f), b * (1.0f - i * 0.1f));
    }
    
    glColor3f(r, g, b);
    drawSphere(position, size, 16, r, g, b);
    
    glColor3f(r * 1.3f, g * 1.3f, b * 1.3f);
    drawSphere(position, size * 0.6f, 12, r * 1.3f, g * 1.3f, b * 1.3f);
    
    glm::vec3 highlightPos = position + glm::vec3(size * 0.4f, size * 0.4f, size * 0.3f);
    glColor3f(1.0f, 1.0f, 1.0f);
    drawSphere(highlightPos, size * 0.2f, 8, 1.0f, 1.0f, 1.0f);
    
    glEnable(GL_LIGHTING);
}

void Renderer::drawGauntletLines(glm::vec3 palmCenter, glm::vec3 handDir, glm::vec3 handRight, glm::vec3 handUp, float halfPalmWidth, float halfPalmThickness, float halfPalmLength)
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    
    glm::vec3 backOfHand = palmCenter + handUp * halfPalmThickness * 0.5f;
    float stoneRadius = 0.35f;
    float stoneHeight = halfPalmThickness * 1.2f;
    
    float angles[6] = {
        0.0f,
        60.0f  * (float)M_PI / 180.0f,
        120.0f * (float)M_PI / 180.0f,
        180.0f * (float)M_PI / 180.0f,
        240.0f * (float)M_PI / 180.0f,
        300.0f * (float)M_PI / 180.0f
    };
    
    glm::vec3 stonePositions[6];
    for (int i = 0; i < 6; i++) {
        float cosAngle = (float)cos(angles[i]);
        float sinAngle = (float)sin(angles[i]);
        stonePositions[i] = backOfHand + 
            handDir * cosAngle * stoneRadius + 
            handRight * sinAngle * stoneRadius -
            handUp * stoneHeight;
    }
    
    glColor3f(0.8f, 0.6f, 0.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < 6; i++) {
        int next = (i + 1) % 6;
        glVertex3f(stonePositions[i].x, stonePositions[i].y, stonePositions[i].z);
        glVertex3f(stonePositions[next].x, stonePositions[next].y, stonePositions[next].z);
    }
    glVertex3f(stonePositions[0].x, stonePositions[0].y, stonePositions[0].z);
    glVertex3f(stonePositions[3].x, stonePositions[3].y, stonePositions[3].z);
    glVertex3f(stonePositions[1].x, stonePositions[1].y, stonePositions[1].z);
    glVertex3f(stonePositions[4].x, stonePositions[4].y, stonePositions[4].z);
    glVertex3f(stonePositions[2].x, stonePositions[2].y, stonePositions[2].z);
    glVertex3f(stonePositions[5].x, stonePositions[5].y, stonePositions[5].z);
    glEnd();
    
    glm::vec3 wristBase = backOfHand + handDir * halfPalmLength * 0.5f;
    glColor3f(0.7f, 0.5f, 0.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < 3; i++) {
        float offsetVal = (float)(i - 1) * halfPalmWidth * 0.4f;
        glm::vec3 offset = handRight * offsetVal;
        glm::vec3 start = wristBase + offset;
        glm::vec3 end = wristBase + offset + handUp * halfPalmThickness * 0.5f;
        glVertex3f(start.x, start.y, start.z);
        glVertex3f(end.x, end.y, end.z);
    }
    glEnd();
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::drawArm(glm::vec3 worldOffset, glm::quat correctedQ,
                        float skinR, float skinG, float skinB, bool mirrorThumb,
                        glm::quat upperArmQ, bool hasUpperArmQ)
{
    // worldOffset is the SHOULDER — the fixed anchor point
    glm::vec3 shoulder = worldOffset;

    // Upper arm hangs downward from shoulder, driven by upperArmQ
    glm::vec3 upperArmDir = hasUpperArmQ
        ? glm::normalize(upperArmQ * glm::vec3(0.0f, -1.0f, 0.0f))
        : glm::vec3(0.0f, -1.0f, 0.0f);

    // Elbow is derived from shoulder + upper arm orientation
    glm::vec3 elbow = shoulder + upperArmDir * 4.0f;

    // Forearm direction driven by correctedQ (forearm sensor), chained off elbow
    glm::vec3 forearmDir = glm::normalize(correctedQ * glm::vec3(0.0f, -1.0f, 0.0f));

    float forearmLength = 4.0f;
    glm::vec3 wrist = elbow + forearmDir * forearmLength;

    // Draw upper arm: shoulder → elbow
    drawSphere  (shoulder, 0.3f,  16, skinR * 0.95f, skinG * 0.95f, skinB * 0.95f);
    drawCylinder(shoulder, elbow, 0.25f, 16, skinR, skinG, skinB);

    // Draw forearm: elbow → wrist
    drawSphere  (elbow, 0.28f, 16, skinR * 0.90f, skinG * 0.90f, skinB * 0.90f);
    drawCylinder(elbow, wrist, 0.22f, 16, skinR * 0.95f, skinG * 0.95f, skinB * 0.95f);
    drawSphere  (wrist, 0.25f, 16, skinR * 0.90f, skinG * 0.90f, skinB * 0.90f);

    // Hand orientation derived from forearm sensor
    glm::vec3 handDir   = glm::normalize(correctedQ * glm::vec3(0.0f, -1.0f, 0.0f));
    glm::vec3 handRight = glm::normalize(correctedQ * glm::vec3(1.0f,  0.0f, 0.0f));
    glm::vec3 handUp    = glm::normalize(correctedQ * glm::vec3(0.0f,  0.0f, 1.0f));

    glm::vec3 palmCenter = wrist + handDir * 0.5f;

    float palmWidth         = 1.2f;
    float palmThickness     = 0.15f;
    float palmLength        = 1.0f;
    float halfPalmWidth     = palmWidth     * 0.5f;
    float halfPalmThickness = palmThickness * 0.5f;
    float halfPalmLength    = palmLength    * 0.5f;

    glm::vec3 palmCorners[8];
    palmCorners[0] = wrist + handRight * halfPalmWidth  + handUp * halfPalmThickness;
    palmCorners[1] = wrist + handRight * halfPalmWidth  - handUp * halfPalmThickness;
    palmCorners[2] = wrist - handRight * halfPalmWidth  + handUp * halfPalmThickness;
    palmCorners[3] = wrist - handRight * halfPalmWidth  - handUp * halfPalmThickness;
    palmCorners[4] = palmCenter + handRight * halfPalmWidth  + handUp * halfPalmThickness;
    palmCorners[5] = palmCenter + handRight * halfPalmWidth  - handUp * halfPalmThickness;
    palmCorners[6] = palmCenter - handRight * halfPalmWidth  + handUp * halfPalmThickness;
    palmCorners[7] = palmCenter - handRight * halfPalmWidth  - handUp * halfPalmThickness;

    glColor3f(skinR * 1.0f, skinG * 0.85f, skinB * 0.70f);

    // Front face
    glBegin(GL_QUADS);
    glm::vec3 normal = handDir;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[4].x, palmCorners[4].y, palmCorners[4].z);
    glVertex3f(palmCorners[5].x, palmCorners[5].y, palmCorners[5].z);
    glVertex3f(palmCorners[7].x, palmCorners[7].y, palmCorners[7].z);
    glVertex3f(palmCorners[6].x, palmCorners[6].y, palmCorners[6].z);
    glEnd();

    // Back face
    glBegin(GL_QUADS);
    normal = -handDir;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[0].x, palmCorners[0].y, palmCorners[0].z);
    glVertex3f(palmCorners[1].x, palmCorners[1].y, palmCorners[1].z);
    glVertex3f(palmCorners[3].x, palmCorners[3].y, palmCorners[3].z);
    glVertex3f(palmCorners[2].x, palmCorners[2].y, palmCorners[2].z);
    glEnd();

    // Top face
    glBegin(GL_QUADS);
    normal = handUp;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[0].x, palmCorners[0].y, palmCorners[0].z);
    glVertex3f(palmCorners[2].x, palmCorners[2].y, palmCorners[2].z);
    glVertex3f(palmCorners[6].x, palmCorners[6].y, palmCorners[6].z);
    glVertex3f(palmCorners[4].x, palmCorners[4].y, palmCorners[4].z);
    glEnd();

    // Bottom face
    glBegin(GL_QUADS);
    normal = -handUp;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[1].x, palmCorners[1].y, palmCorners[1].z);
    glVertex3f(palmCorners[3].x, palmCorners[3].y, palmCorners[3].z);
    glVertex3f(palmCorners[7].x, palmCorners[7].y, palmCorners[7].z);
    glVertex3f(palmCorners[5].x, palmCorners[5].y, palmCorners[5].z);
    glEnd();

    // Right face
    glBegin(GL_QUADS);
    normal = handRight;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[0].x, palmCorners[0].y, palmCorners[0].z);
    glVertex3f(palmCorners[4].x, palmCorners[4].y, palmCorners[4].z);
    glVertex3f(palmCorners[5].x, palmCorners[5].y, palmCorners[5].z);
    glVertex3f(palmCorners[1].x, palmCorners[1].y, palmCorners[1].z);
    glEnd();

    // Left face
    glBegin(GL_QUADS);
    normal = -handRight;
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(palmCorners[2].x, palmCorners[2].y, palmCorners[2].z);
    glVertex3f(palmCorners[6].x, palmCorners[6].y, palmCorners[6].z);
    glVertex3f(palmCorners[7].x, palmCorners[7].y, palmCorners[7].z);
    glVertex3f(palmCorners[3].x, palmCorners[3].y, palmCorners[3].z);
    glEnd();

    // ============================================================
    // DRAW INFINITY GAUNTLET (Stones + Lines on back of hand)
    // ============================================================

    drawGauntletLines(palmCenter, handDir, handRight, handUp, halfPalmWidth, halfPalmThickness, halfPalmLength);

    glm::vec3 backOfHand = palmCenter - handUp * halfPalmThickness * 0.5f;
    float stoneRadius = 0.35f;
    float stoneHeight = halfPalmThickness * 1.2f;

    float angles[6] = {
        0.0f,
        60.0f  * (float)M_PI / 180.0f,
        120.0f * (float)M_PI / 180.0f,
        180.0f * (float)M_PI / 180.0f,
        240.0f * (float)M_PI / 180.0f,
        300.0f * (float)M_PI / 180.0f
    };

    float stoneColors[6][3] = {
        {0.0f,  1.0f,  0.0f},
        {1.0f,  0.84f, 0.0f},
        {0.0f,  0.0f,  1.0f},
        {1.0f,  0.0f,  0.0f},
        {0.58f, 0.0f,  0.83f},
        {1.0f,  0.5f,  0.0f}
    };

    glm::vec3 stonePositions[6];
    for (int i = 0; i < 6; i++) {
        float cosAngle = (float)cos(angles[i]);
        float sinAngle = (float)sin(angles[i]);
        stonePositions[i] = backOfHand +
            handDir   * cosAngle * stoneRadius +
            handRight * sinAngle * stoneRadius -
            handUp    * stoneHeight;

        drawInfinityStone(stonePositions[i], 0.08f,
            stoneColors[i][0], stoneColors[i][1], stoneColors[i][2]);
    }

    // Draw fingers
    float fingerSpread = 0.22f;

    glm::vec3 indexBase  = palmCenter + handRight * fingerSpread * 2.0f + handDir * halfPalmLength;
    drawFinger(indexBase, handDir, 0.08f, skinR * 0.95f, skinG * 0.82f, skinB * 0.65f);

    glm::vec3 middleBase = palmCenter + handDir * halfPalmLength;
    drawFinger(middleBase, handDir, 0.09f, skinR * 0.95f, skinG * 0.82f, skinB * 0.65f);

    glm::vec3 ringBase   = palmCenter - handRight * fingerSpread * 1.8f + handDir * halfPalmLength;
    drawFinger(ringBase, handDir, 0.08f, skinR * 0.95f, skinG * 0.82f, skinB * 0.65f);

    glm::vec3 pinkyBase  = palmCenter - handRight * fingerSpread * 3.6f + handDir * halfPalmLength;
    drawFinger(pinkyBase, handDir * 0.95f + handRight * 0.1f, 0.07f, skinR * 0.95f, skinG * 0.82f, skinB * 0.65f);

    float thumbSide = mirrorThumb ? -1.0f : 1.0f;
    glm::vec3 thumbBase = wrist + handRight * halfPalmWidth * 0.8f * thumbSide + handDir * 0.3f;
    glm::vec3 thumbDir  = glm::normalize(handDir * 0.5f + handRight * 0.7f * thumbSide + handUp * 0.3f);
    drawFinger(thumbBase, thumbDir, 0.1f, skinR * 1.0f, skinG * 0.82f, skinB * 0.65f);

    // Local sensor axes drawn at elbow joint
    glDisable(GL_LIGHTING);
    glm::vec3 localX = correctedQ * glm::vec3(-1.0f, 0.0f, 0.0f);
    glm::vec3 localY = correctedQ * glm::vec3( 0.0f, 1.0f, 0.0f);
    glm::vec3 localZ = correctedQ * glm::vec3( 0.0f, 0.0f, 1.0f);

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(elbow.x, elbow.y, elbow.z);
    glVertex3f(elbow.x + localX.x * 1.5f, elbow.y + localX.y * 1.5f, elbow.z + localX.z * 1.5f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(elbow.x, elbow.y, elbow.z);
    glVertex3f(elbow.x + localY.x * 1.5f, elbow.y + localY.y * 1.5f, elbow.z + localY.z * 1.5f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(elbow.x, elbow.y, elbow.z);
    glVertex3f(elbow.x + localZ.x * 1.5f, elbow.y + localZ.y * 1.5f, elbow.z + localZ.z * 1.5f);
    glEnd();
    glEnable(GL_LIGHTING);
}