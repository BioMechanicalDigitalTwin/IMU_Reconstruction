#include "sensor_manager.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuat1(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat2(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat3(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat4(1.0f, 0.0f, 0.0f, 0.0f)
    , quaternionMode(0)
    , calibrationReference1(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected1(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected1(false)
    , calibrationReference2(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected2(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected2(false)
    , calibrationReference3(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected3(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected3(false)
    , calibrationReference4(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected4(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected4(false)
{
    std::cout << "Quaternion mode 1/4. Press M to cycle modes, then recalibrate with C/V/B/N.\n";
}

void SensorManager::updateSensorQuat(glm::quat& current, const glm::quat& incoming) const
{
    glm::quat target = glm::normalize(incoming);

    if (glm::dot(current, target) < 0.0f) {
        target = -target;
    }

    current = target;
}

// --- Setters ---

void SensorManager::setHipsQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex1);
    updateSensorQuat(sensorQuat1, q);
}

void SensorManager::setChestQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex2);
    updateSensorQuat(sensorQuat2, q);
}

void SensorManager::setLUAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex3);
    updateSensorQuat(sensorQuat3, q);
}

void SensorManager::setLFAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex4);
    updateSensorQuat(sensorQuat4, q);
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
    std::lock_guard<std::mutex> lock(calibMutex1);
    calibrationReference1 = glm::normalize(q);
    hasSmoothedCorrected1 = false;
    std::cout << "Calibrated HIPS\n";
}

void SensorManager::calibrateChest()
{
    glm::quat q = getChestQuat();
    std::lock_guard<std::mutex> lock(calibMutex2);
    calibrationReference2 = glm::normalize(q);
    hasSmoothedCorrected2 = false;
    std::cout << "Calibrated CHEST\n";
}

void SensorManager::calibrateLUA()
{
    glm::quat q = getLUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex3);
    calibrationReference3 = glm::normalize(q);
    hasSmoothedCorrected3 = false;
    std::cout << "Calibrated L_UA\n";
}

void SensorManager::calibrateLFA()
{
    glm::quat q = getLFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex4);
    calibrationReference4 = glm::normalize(q);
    hasSmoothedCorrected4 = false;
    std::cout << "Calibrated L_FA\n";
}

void SensorManager::toggleQuaternionConvention()
{
    int mode = (quaternionMode.load() + 1) % 4;
    quaternionMode.store(mode);
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4);
    hasSmoothedCorrected1 = false;
    hasSmoothedCorrected2 = false;
    hasSmoothedCorrected3 = false;
    hasSmoothedCorrected4 = false;
    std::cout << "Quaternion mode " << (mode + 1) << "/4. Recalibrate with C/V/B/N.\n";
}

// --- Corrected getters ---

glm::quat SensorManager::neutralPose() const
{
    return glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::quat SensorManager::computeMotionDelta(const glm::quat& sensorQuat,
                                            const glm::quat& calibrationReference) const
{
    glm::quat current = glm::normalize(sensorQuat);
    glm::quat reference = glm::normalize(calibrationReference);

    if (glm::dot(reference, current) < 0.0f) {
        current = -current;
    }

    switch (quaternionMode.load()) {
    case 0:
        return glm::normalize(current * glm::inverse(reference));
    case 1:
        return glm::normalize(glm::inverse(reference) * current);
    case 2:
        return glm::normalize(glm::conjugate(current) * glm::inverse(glm::conjugate(reference)));
    default:
        return glm::normalize(glm::inverse(glm::conjugate(reference)) * glm::conjugate(current));
    }
}

glm::quat SensorManager::computeCorrectedQuat(const glm::quat& sensorQuat,
                                               const glm::quat& calibrationReference) const
{
    glm::quat delta = computeMotionDelta(sensorQuat, calibrationReference);
    glm::quat q = glm::normalize(delta * neutralPose());
    q.y = -q.y;
    return glm::normalize(q);
}

glm::quat SensorManager::smoothCorrectedQuat(glm::quat& current,
                                             bool& initialized,
                                             const glm::quat& target) const
{
    glm::quat normalizedTarget = glm::normalize(target);

    if (!initialized) {
        current = normalizedTarget;
        initialized = true;
        return current;
    }

    if (glm::dot(current, normalizedTarget) < 0.0f) {
        normalizedTarget = -normalizedTarget;
    }

    const float dot = std::clamp(glm::dot(current, normalizedTarget), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(std::abs(dot));

    if (angle < kDeadbandRadians) {
        return current;
    }

    const float alpha = angle > kFastMotionRadians ? kFastSmoothingAlpha : kSmoothingAlpha;
    current = glm::normalize(glm::slerp(current, normalizedTarget, alpha));
    return current;
}

glm::quat SensorManager::getCorrectedHipsQuat() const
{
    glm::quat q = getHipsQuat();
    std::lock_guard<std::mutex> lock(calibMutex1);
    return smoothCorrectedQuat(smoothedCorrected1, hasSmoothedCorrected1,
                               computeCorrectedQuat(q, calibrationReference1));
}

glm::quat SensorManager::getCorrectedChestQuat() const
{
    glm::quat q = getChestQuat();
    std::lock_guard<std::mutex> lock(calibMutex2);
    return smoothCorrectedQuat(smoothedCorrected2, hasSmoothedCorrected2,
                               computeCorrectedQuat(q, calibrationReference2));
}

glm::quat SensorManager::getCorrectedLUAQuat() const
{
    glm::quat q = getLUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex3);
    return smoothCorrectedQuat(smoothedCorrected3, hasSmoothedCorrected3,
                               computeCorrectedQuat(q, calibrationReference3));
}

glm::quat SensorManager::getCorrectedLFAQuat() const
{
    glm::quat q = getLFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex4);
    return smoothCorrectedQuat(smoothedCorrected4, hasSmoothedCorrected4,
                               computeCorrectedQuat(q, calibrationReference4));
}
