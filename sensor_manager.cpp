#include "sensor_manager.h"
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuat1(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat2(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat3(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat4(1.0f, 0.0f, 0.0f, 0.0f)
    , calibrationOffset1(1.0f, 0.0f, 0.0f, 0.0f)
    , calibrationOffset2(1.0f, 0.0f, 0.0f, 0.0f)
    , calibrationOffset3(1.0f, 0.0f, 0.0f, 0.0f)
    , calibrationOffset4(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorOffset(glm::angleAxis(glm::radians(-45.0f), glm::vec3(0, 0, 1)))
{
}

// --- Setters ---

void SensorManager::setHipsQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex1);
    sensorQuat1 = glm::normalize(glm::slerp(sensorQuat1, q, 0.3f));
}

void SensorManager::setChestQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex2);
    sensorQuat2 = glm::normalize(glm::slerp(sensorQuat2, q, 0.3f));
}

void SensorManager::setLUAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex3);
    sensorQuat3 = glm::normalize(glm::slerp(sensorQuat3, q, 0.3f));
}

void SensorManager::setLFAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex4);
    sensorQuat4 = glm::normalize(glm::slerp(sensorQuat4, q, 0.3f));
}

// --- Getters ---

glm::quat SensorManager::getHipsQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex1);
    return sensorQuat1;
}

glm::quat SensorManager::getChestQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex2);
    return sensorQuat2;
}

glm::quat SensorManager::getLUAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex3);
    return sensorQuat3;
}

glm::quat SensorManager::getLFAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex4);
    return sensorQuat4;
}

// --- Calibration ---

void SensorManager::calibrateHips()
{
    glm::quat q = getHipsQuat();
    glm::quat correctedQ = glm::conjugate(q) * sensorOffset;
    glm::quat flip = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    std::lock_guard<std::mutex> lock(calibMutex1);
    calibrationOffset1 = glm::inverse(correctedQ) * flip;
}

void SensorManager::calibrateChest()
{
    glm::quat q = getChestQuat();
    glm::quat correctedQ = glm::conjugate(q) * sensorOffset;
    glm::quat flip = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    std::lock_guard<std::mutex> lock(calibMutex2);
    calibrationOffset2 = glm::inverse(correctedQ) * flip;
}

void SensorManager::calibrateLUA()
{
    glm::quat q = getLUAQuat();
    glm::quat correctedQ = glm::conjugate(q) * sensorOffset;
    glm::quat flip = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    std::lock_guard<std::mutex> lock(calibMutex3);
    calibrationOffset3 = glm::inverse(correctedQ) * flip;
}

void SensorManager::calibrateLFA()
{
    glm::quat q = getLFAQuat();
    glm::quat correctedQ = glm::conjugate(q) * sensorOffset;
    glm::quat flip = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    std::lock_guard<std::mutex> lock(calibMutex4);
    calibrationOffset4 = glm::inverse(correctedQ) * flip;
}

// --- Corrected getters ---

glm::quat SensorManager::computeCorrectedQuat(const glm::quat& sensorQuat,
                                               const glm::quat& calibOffset) const
{
    glm::quat q = (glm::conjugate(sensorQuat) * sensorOffset) * calibOffset;
    q.y = -q.y;
    return glm::normalize(q);
}

glm::quat SensorManager::getCorrectedHipsQuat() const
{
    glm::quat q = getHipsQuat();
    std::lock_guard<std::mutex> lock(calibMutex1);
    return computeCorrectedQuat(q, calibrationOffset1);
}

glm::quat SensorManager::getCorrectedChestQuat() const
{
    glm::quat q = getChestQuat();
    std::lock_guard<std::mutex> lock(calibMutex2);
    return computeCorrectedQuat(q, calibrationOffset2);
}

glm::quat SensorManager::getCorrectedLUAQuat() const
{
    glm::quat q = getLUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex3);
    return computeCorrectedQuat(q, calibrationOffset3);
}

glm::quat SensorManager::getCorrectedLFAQuat() const
{
    glm::quat q = getLFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex4);
    return computeCorrectedQuat(q, calibrationOffset4);
}