#include "sensor_manager.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuatLFA(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuatRFA(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat3(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuatRUA(1.0f, 0.0f, 0.0f, 0.0f)
    , quaternionMode(0)
    , calibrationReferenceLFA(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrectedLFA(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrectedLFA(false)
    , calibrationReferenceRFA(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrectedRFA(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrectedRFA(false)
    , calibrationReference3(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected3(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected3(false)
    , calibrationReferenceRUA(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrectedRUA(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrectedRUA(false)
    , sensorQuat5(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat6(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat7(1.0f, 0.0f, 0.0f, 0.0f)
    , sensorQuat8(1.0f, 0.0f, 0.0f, 0.0f)
    , calibrationReference5(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected5(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected5(false)
    , calibrationReference6(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected6(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected6(false)
    , calibrationReference7(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected7(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected7(false)
    , calibrationReference8(1.0f, 0.0f, 0.0f, 0.0f)
    , smoothedCorrected8(1.0f, 0.0f, 0.0f, 0.0f)
    , hasSmoothedCorrected8(false)
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

void SensorManager::setLFAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex1);
    updateSensorQuat(sensorQuatLFA, q);
}

void SensorManager::setRFAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex2);
    updateSensorQuat(sensorQuatRFA, q);
}

void SensorManager::setLUAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex3);
    updateSensorQuat(sensorQuat3, q);
}

void SensorManager::setRUAQuat(const glm::quat& q)
{
    std::lock_guard<std::mutex> lock(quatMutex4);
    updateSensorQuat(sensorQuatRUA, q);
}

// --- Getters ---

glm::quat SensorManager::getLFAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex1);
    return sensorQuatLFA;
}

glm::quat SensorManager::getRFAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex2);
    return sensorQuatRFA;
}

glm::quat SensorManager::getLUAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex3);
    return sensorQuat3;
}

glm::quat SensorManager::getRUAQuat() const
{
    std::lock_guard<std::mutex> lock(quatMutex4);
    return sensorQuatRUA;
}

// --- Calibration ---

void SensorManager::calibrateLFA()
{
    glm::quat q = getLFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex1);
    calibrationReferenceLFA = glm::normalize(q);
    hasSmoothedCorrectedLFA = false;
    std::cout << "Calibrated L_FA\n";
}

void SensorManager::calibrateRFA()
{
    glm::quat q = getRFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex2);
    calibrationReferenceRFA = glm::normalize(q);
    hasSmoothedCorrectedRFA = false;
    std::cout << "Calibrated R_FA\n";
}

void SensorManager::calibrateLUA()
{
    glm::quat q = getLUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex3);
    calibrationReference3 = glm::normalize(q);
    hasSmoothedCorrected3 = false;
    std::cout << "Calibrated L_UA\n";
}

void SensorManager::calibrateRUA()
{
    glm::quat q = getRUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex4);
    calibrationReferenceRUA = glm::normalize(q);
    hasSmoothedCorrectedRUA = false;
    std::cout << "Calibrated R_UA\n";
}

void SensorManager::toggleQuaternionConvention()
{
    int mode = (quaternionMode.load() + 1) % 4;
    quaternionMode.store(mode);
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4);
    hasSmoothedCorrectedLFA = false;
    hasSmoothedCorrectedRFA = false;
    hasSmoothedCorrected3 = false;
    hasSmoothedCorrectedRUA = false;
    std::cout << "Quaternion mode " << (mode + 1) << "/4. Recalibrate with C/V/B/N.\n";
}

void SensorManager::setLTHQuat(const glm::quat& q) { std::lock_guard<std::mutex> lock(quatMutex5); updateSensorQuat(sensorQuat5, q); }
void SensorManager::setLSHQuat(const glm::quat& q) { std::lock_guard<std::mutex> lock(quatMutex6); updateSensorQuat(sensorQuat6, q); }
void SensorManager::setRTHQuat(const glm::quat& q) { std::lock_guard<std::mutex> lock(quatMutex7); updateSensorQuat(sensorQuat7, q); }
void SensorManager::setRSHQuat(const glm::quat& q) { std::lock_guard<std::mutex> lock(quatMutex8); updateSensorQuat(sensorQuat8, q); }

glm::quat SensorManager::getLTHQuat() const { std::lock_guard<std::mutex> lock(quatMutex5); return sensorQuat5; }
glm::quat SensorManager::getLSHQuat() const { std::lock_guard<std::mutex> lock(quatMutex6); return sensorQuat6; }
glm::quat SensorManager::getRTHQuat() const { std::lock_guard<std::mutex> lock(quatMutex7); return sensorQuat7; }
glm::quat SensorManager::getRSHQuat() const { std::lock_guard<std::mutex> lock(quatMutex8); return sensorQuat8; }

void SensorManager::calibrateLTH() { glm::quat q = getLTHQuat(); std::lock_guard<std::mutex> lock(calibMutex5); calibrationReference5 = glm::normalize(q); hasSmoothedCorrected5 = false; std::cout << "Calibrated L_TH\n"; }
void SensorManager::calibrateLSH() { glm::quat q = getLSHQuat(); std::lock_guard<std::mutex> lock(calibMutex6); calibrationReference6 = glm::normalize(q); hasSmoothedCorrected6 = false; std::cout << "Calibrated L_SH\n"; }
void SensorManager::calibrateRTH() { glm::quat q = getRTHQuat(); std::lock_guard<std::mutex> lock(calibMutex7); calibrationReference7 = glm::normalize(q); hasSmoothedCorrected7 = false; std::cout << "Calibrated R_TH\n"; }
void SensorManager::calibrateRSH() { glm::quat q = getRSHQuat(); std::lock_guard<std::mutex> lock(calibMutex8); calibrationReference8 = glm::normalize(q); hasSmoothedCorrected8 = false; std::cout << "Calibrated R_SH\n"; }

glm::quat SensorManager::getCorrectedLTHQuat() const { glm::quat q = getLTHQuat(); std::lock_guard<std::mutex> lock(calibMutex5); return smoothCorrectedQuat(smoothedCorrected5, hasSmoothedCorrected5, computeCorrectedQuat(q, calibrationReference5)); }
glm::quat SensorManager::getCorrectedLSHQuat() const { glm::quat q = getLSHQuat(); std::lock_guard<std::mutex> lock(calibMutex6); return smoothCorrectedQuat(smoothedCorrected6, hasSmoothedCorrected6, computeCorrectedQuat(q, calibrationReference6)); }
glm::quat SensorManager::getCorrectedRTHQuat() const { glm::quat q = getRTHQuat(); std::lock_guard<std::mutex> lock(calibMutex7); return smoothCorrectedQuat(smoothedCorrected7, hasSmoothedCorrected7, computeCorrectedQuat(q, calibrationReference7)); }
glm::quat SensorManager::getCorrectedRSHQuat() const { glm::quat q = getRSHQuat(); std::lock_guard<std::mutex> lock(calibMutex8); return smoothCorrectedQuat(smoothedCorrected8, hasSmoothedCorrected8, computeCorrectedQuat(q, calibrationReference8)); }

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

glm::quat SensorManager::getCorrectedLFAQuat() const
{
    glm::quat q = getLFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex1);
    return smoothCorrectedQuat(smoothedCorrectedLFA, hasSmoothedCorrectedLFA,
                               computeCorrectedQuat(q, calibrationReferenceLFA));
}

glm::quat SensorManager::getCorrectedRFAQuat() const
{
    glm::quat q = getRFAQuat();
    std::lock_guard<std::mutex> lock(calibMutex2);
    return smoothCorrectedQuat(smoothedCorrectedRFA, hasSmoothedCorrectedRFA,
                               computeCorrectedQuat(q, calibrationReferenceRFA));
}

glm::quat SensorManager::getCorrectedLUAQuat() const
{
    glm::quat q = getLUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex3);
    return smoothCorrectedQuat(smoothedCorrected3, hasSmoothedCorrected3,
                               computeCorrectedQuat(q, calibrationReference3));
}

glm::quat SensorManager::getCorrectedRUAQuat() const
{
    glm::quat q = getRUAQuat();
    std::lock_guard<std::mutex> lock(calibMutex4);
    return smoothCorrectedQuat(smoothedCorrectedRUA, hasSmoothedCorrectedRUA,
                               computeCorrectedQuat(q, calibrationReferenceRUA));
}