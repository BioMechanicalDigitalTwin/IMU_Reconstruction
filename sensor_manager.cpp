#include "sensor_manager.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuatLFA(1,0,0,0), sensorQuatRFA(1,0,0,0)
    , sensorQuat3(1,0,0,0),   sensorQuatRUA(1,0,0,0)
    , sensorQuat5(1,0,0,0),   sensorQuat6(1,0,0,0)
    , sensorQuat7(1,0,0,0),   sensorQuat8(1,0,0,0)
    , sensorQuat9(1,0,0,0),   sensorQuat10(1,0,0,0)
    , quaternionMode(0)
    , calibrationReferenceLFA(1,0,0,0), smoothedCorrectedLFA(1,0,0,0)
    , hasSmoothedCorrectedLFA(false),   lastQLFA(1,0,0,0), stationaryTimerLFA(0)
    , calibrationReferenceRFA(1,0,0,0), smoothedCorrectedRFA(1,0,0,0)
    , hasSmoothedCorrectedRFA(false),   lastQRFA(1,0,0,0), stationaryTimerRFA(0)
    , calibrationReference3(1,0,0,0),   smoothedCorrected3(1,0,0,0)
    , hasSmoothedCorrected3(false),     lastQ3(1,0,0,0), stationaryTimer3(0)
    , calibrationReferenceRUA(1,0,0,0), smoothedCorrectedRUA(1,0,0,0)
    , hasSmoothedCorrectedRUA(false),   lastQRUA(1,0,0,0), stationaryTimerRUA(0)
    , calibrationReference5(1,0,0,0),   smoothedCorrected5(1,0,0,0)
    , hasSmoothedCorrected5(false),     lastQ5(1,0,0,0), stationaryTimer5(0)
    , calibrationReference6(1,0,0,0),   smoothedCorrected6(1,0,0,0)
    , hasSmoothedCorrected6(false),     lastQ6(1,0,0,0), stationaryTimer6(0)
    , calibrationReference7(1,0,0,0),   smoothedCorrected7(1,0,0,0)
    , hasSmoothedCorrected7(false),     lastQ7(1,0,0,0), stationaryTimer7(0)
    , calibrationReference8(1,0,0,0),   smoothedCorrected8(1,0,0,0)
    , hasSmoothedCorrected8(false),     lastQ8(1,0,0,0), stationaryTimer8(0)
    , calibrationReference9(1,0,0,0),   smoothedCorrected9(1,0,0,0)
    , hasSmoothedCorrected9(false),     lastQ9(1,0,0,0), stationaryTimer9(0)
    , calibrationReference10(1,0,0,0),  smoothedCorrected10(1,0,0,0)
    , hasSmoothedCorrected10(false),    lastQ10(1,0,0,0), stationaryTimer10(0)
{
    std::cout << "Quaternion mode 1/4. Press M to cycle, then recalibrate.\n";
}

void SensorManager::updateSensorQuat(glm::quat& current, const glm::quat& incoming) const
{
    glm::quat t = glm::normalize(incoming);
    if (glm::dot(current, t) < 0.0f) t = -t;
    current = t;
}

glm::quat SensorManager::neutralPose() const
{
    return glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::quat SensorManager::computeMotionDelta(const glm::quat& sensorQuat,
                                             const glm::quat& calibrationReference) const
{
    glm::quat current   = glm::normalize(sensorQuat);
    glm::quat reference = glm::normalize(calibrationReference);
    if (glm::dot(reference, current) < 0.0f) current = -current;

    switch (quaternionMode.load()) {
    case 0:  return glm::normalize(current * glm::inverse(reference));
    case 1:  return glm::normalize(glm::inverse(reference) * current);
    case 2:  return glm::normalize(glm::conjugate(current) * glm::inverse(glm::conjugate(reference)));
    default: return glm::normalize(glm::inverse(glm::conjugate(reference)) * glm::conjugate(current));
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
    glm::quat nt = glm::normalize(target);
    if (!initialized) { current = nt; initialized = true; return current; }
    if (glm::dot(current, nt) < 0.0f) nt = -nt;

    const float dot   = std::clamp(glm::dot(current, nt), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(std::abs(dot));

    // Deadband: ignore sub-threshold jitter
    if (angle < kDeadbandRadians) return current;

    const float t     = glm::smoothstep(0.02f, 0.25f, angle);  // was (0.05, 0.35)
    const float alpha = glm::mix(kSmoothingAlpha, kFastSmoothingAlpha, t);
    current = glm::normalize(glm::slerp(current, nt, alpha));
    return current;
}

void SensorManager::autoRecalibrate(glm::quat& calibRef,
                                     glm::quat& lastQ,
                                     float& stationaryTimer,
                                     const glm::quat& current) const
{
    const float angle = 2.0f * std::acos(
        std::clamp(std::abs(glm::dot(lastQ, current)), 0.0f, 1.0f));

    if (angle < kStationaryThreshold) {
        stationaryTimer += 16.0f;
        if (stationaryTimer > kStationaryTimeMs) {
            const float refDrift = 2.0f * std::acos(
                std::clamp(std::abs(glm::dot(calibRef, current)), 0.0f, 1.0f));
            // Only nudge if drift is small — don't chase a raised arm
            if (refDrift < glm::radians(15.0f)) {
                calibRef = glm::normalize(glm::slerp(calibRef, current, kDriftCorrAlpha));
            }
            // Don't reset timer — keep checking but at reduced rate
            stationaryTimer = kStationaryTimeMs - 500.0f;
        }
    } else {
        stationaryTimer = 0.0f;
    }
    lastQ = current;
}

void SensorManager::setLFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex1);  updateSensorQuat(sensorQuatLFA, q); }
void SensorManager::setRFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex2);  updateSensorQuat(sensorQuatRFA, q); }
void SensorManager::setLUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex3);  updateSensorQuat(sensorQuat3,   q); }
void SensorManager::setRUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex4);  updateSensorQuat(sensorQuatRUA, q); }
void SensorManager::setLTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex5);  updateSensorQuat(sensorQuat5,   q); }
void SensorManager::setLSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex6);  updateSensorQuat(sensorQuat6,   q); }
void SensorManager::setRTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex7);  updateSensorQuat(sensorQuat7,   q); }
void SensorManager::setRSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex8);  updateSensorQuat(sensorQuat8,   q); }
void SensorManager::setHipsQuat(const glm::quat& q) { std::lock_guard<std::mutex> l(quatMutex9);  updateSensorQuat(sensorQuat9,   q); }
void SensorManager::setChestQuat(const glm::quat& q){ std::lock_guard<std::mutex> l(quatMutex10); updateSensorQuat(sensorQuat10,  q); }

glm::quat SensorManager::getLFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex1);  return sensorQuatLFA; }
glm::quat SensorManager::getRFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex2);  return sensorQuatRFA; }
glm::quat SensorManager::getLUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex3);  return sensorQuat3;   }
glm::quat SensorManager::getRUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex4);  return sensorQuatRUA; }
glm::quat SensorManager::getLTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex5);  return sensorQuat5;   }
glm::quat SensorManager::getLSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex6);  return sensorQuat6;   }
glm::quat SensorManager::getRTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex7);  return sensorQuat7;   }
glm::quat SensorManager::getRSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex8);  return sensorQuat8;   }
glm::quat SensorManager::getHipsQuat() const { std::lock_guard<std::mutex> l(quatMutex9);  return sensorQuat9;   }
glm::quat SensorManager::getChestQuat()const { std::lock_guard<std::mutex> l(quatMutex10); return sensorQuat10;  }

void SensorManager::calibrateLFA()  { auto q = getLFAQuat();  std::lock_guard<std::mutex> l(calibMutex1);  calibrationReferenceLFA = glm::normalize(q); hasSmoothedCorrectedLFA = false; stationaryTimerLFA = 0; std::cout << "Calibrated L_FA\n"; }
void SensorManager::calibrateRFA()  { auto q = getRFAQuat();  std::lock_guard<std::mutex> l(calibMutex2);  calibrationReferenceRFA = glm::normalize(q); hasSmoothedCorrectedRFA = false; stationaryTimerRFA = 0; std::cout << "Calibrated R_FA\n"; }
void SensorManager::calibrateLUA()  { auto q = getLUAQuat();  std::lock_guard<std::mutex> l(calibMutex3);  calibrationReference3   = glm::normalize(q); hasSmoothedCorrected3   = false; stationaryTimer3   = 0; std::cout << "Calibrated L_UA\n"; }
void SensorManager::calibrateRUA()  { auto q = getRUAQuat();  std::lock_guard<std::mutex> l(calibMutex4);  calibrationReferenceRUA = glm::normalize(q); hasSmoothedCorrectedRUA = false; stationaryTimerRUA = 0; std::cout << "Calibrated R_UA\n"; }
void SensorManager::calibrateLTH()  { auto q = getLTHQuat();  std::lock_guard<std::mutex> l(calibMutex5);  calibrationReference5   = glm::normalize(q); hasSmoothedCorrected5   = false; stationaryTimer5   = 0; std::cout << "Calibrated L_TH\n"; }
void SensorManager::calibrateLSH()  { auto q = getLSHQuat();  std::lock_guard<std::mutex> l(calibMutex6);  calibrationReference6   = glm::normalize(q); hasSmoothedCorrected6   = false; stationaryTimer6   = 0; std::cout << "Calibrated L_SH\n"; }
void SensorManager::calibrateRTH()  { auto q = getRTHQuat();  std::lock_guard<std::mutex> l(calibMutex7);  calibrationReference7   = glm::normalize(q); hasSmoothedCorrected7   = false; stationaryTimer7   = 0; std::cout << "Calibrated R_TH\n"; }
void SensorManager::calibrateRSH()  { auto q = getRSHQuat();  std::lock_guard<std::mutex> l(calibMutex8);  calibrationReference8   = glm::normalize(q); hasSmoothedCorrected8   = false; stationaryTimer8   = 0; std::cout << "Calibrated R_SH\n"; }
void SensorManager::calibrateHips() { auto q = getHipsQuat(); std::lock_guard<std::mutex> l(calibMutex9);  calibrationReference9   = glm::normalize(q); hasSmoothedCorrected9   = false; stationaryTimer9   = 0; std::cout << "Calibrated HIPS\n"; }
void SensorManager::calibrateChest(){ auto q = getChestQuat();std::lock_guard<std::mutex> l(calibMutex10); calibrationReference10  = glm::normalize(q); hasSmoothedCorrected10  = false; stationaryTimer10  = 0; std::cout << "Calibrated CHEST\n"; }

void SensorManager::toggleQuaternionConvention()
{
    int mode = (quaternionMode.load() + 1) % 4;
    quaternionMode.store(mode);
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4);
    hasSmoothedCorrectedLFA = hasSmoothedCorrectedRFA = false;
    hasSmoothedCorrected3   = hasSmoothedCorrectedRUA = false;
    std::cout << "Quaternion mode " << (mode + 1) << "/4. Recalibrate.\n";
}

glm::quat SensorManager::getCorrectedLFAQuat() const {
    auto q = getLFAQuat();
    std::lock_guard<std::mutex> l(calibMutex1);
    autoRecalibrate(calibrationReferenceLFA, lastQLFA, stationaryTimerLFA, q);
    return smoothCorrectedQuat(smoothedCorrectedLFA, hasSmoothedCorrectedLFA, computeCorrectedQuat(q, calibrationReferenceLFA));
}
glm::quat SensorManager::getCorrectedRFAQuat() const {
    auto q = getRFAQuat();
    std::lock_guard<std::mutex> l(calibMutex2);
    autoRecalibrate(calibrationReferenceRFA, lastQRFA, stationaryTimerRFA, q);
    return smoothCorrectedQuat(smoothedCorrectedRFA, hasSmoothedCorrectedRFA, computeCorrectedQuat(q, calibrationReferenceRFA));
}
glm::quat SensorManager::getCorrectedLUAQuat() const {
    auto q = getLUAQuat();
    std::lock_guard<std::mutex> l(calibMutex3);
    autoRecalibrate(calibrationReference3, lastQ3, stationaryTimer3, q);
    return smoothCorrectedQuat(smoothedCorrected3, hasSmoothedCorrected3, computeCorrectedQuat(q, calibrationReference3));
}
glm::quat SensorManager::getCorrectedRUAQuat() const {
    auto q = getRUAQuat();
    std::lock_guard<std::mutex> l(calibMutex4);
    autoRecalibrate(calibrationReferenceRUA, lastQRUA, stationaryTimerRUA, q);
    return smoothCorrectedQuat(smoothedCorrectedRUA, hasSmoothedCorrectedRUA, computeCorrectedQuat(q, calibrationReferenceRUA));
}
glm::quat SensorManager::getCorrectedLTHQuat() const {
    auto q = getLTHQuat();
    std::lock_guard<std::mutex> l(calibMutex5);
    autoRecalibrate(calibrationReference5, lastQ5, stationaryTimer5, q);
    return smoothCorrectedQuat(smoothedCorrected5, hasSmoothedCorrected5, computeCorrectedQuat(q, calibrationReference5));
}
glm::quat SensorManager::getCorrectedLSHQuat() const {
    auto q = getLSHQuat();
    std::lock_guard<std::mutex> l(calibMutex6);
    autoRecalibrate(calibrationReference6, lastQ6, stationaryTimer6, q);
    return smoothCorrectedQuat(smoothedCorrected6, hasSmoothedCorrected6, computeCorrectedQuat(q, calibrationReference6));
}
glm::quat SensorManager::getCorrectedRTHQuat() const {
    auto q = getRTHQuat();
    std::lock_guard<std::mutex> l(calibMutex7);
    autoRecalibrate(calibrationReference7, lastQ7, stationaryTimer7, q);
    return smoothCorrectedQuat(smoothedCorrected7, hasSmoothedCorrected7, computeCorrectedQuat(q, calibrationReference7));
}
glm::quat SensorManager::getCorrectedRSHQuat() const {
    auto q = getRSHQuat();
    std::lock_guard<std::mutex> l(calibMutex8);
    autoRecalibrate(calibrationReference8, lastQ8, stationaryTimer8, q);
    return smoothCorrectedQuat(smoothedCorrected8, hasSmoothedCorrected8, computeCorrectedQuat(q, calibrationReference8));
}
glm::quat SensorManager::getCorrectedHipsQuat() const {
    auto q = getHipsQuat();
    std::lock_guard<std::mutex> l(calibMutex9);
    autoRecalibrate(calibrationReference9, lastQ9, stationaryTimer9, q);
    return smoothCorrectedQuat(smoothedCorrected9, hasSmoothedCorrected9, computeCorrectedQuat(q, calibrationReference9));
}
glm::quat SensorManager::getCorrectedChestQuat() const {
    auto q = getChestQuat();
    std::lock_guard<std::mutex> l(calibMutex10);
    autoRecalibrate(calibrationReference10, lastQ10, stationaryTimer10, q);
    return smoothCorrectedQuat(smoothedCorrected10, hasSmoothedCorrected10, computeCorrectedQuat(q, calibrationReference10));
}