#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <mutex>

class SensorManager {
public:
    SensorManager();
    
    void setLFAQuat(const glm::quat& q);   // was HIPS
    void setRFAQuat(const glm::quat& q);   // was CHEST
    void setLUAQuat(const glm::quat& q);
    void setRUAQuat(const glm::quat& q);   // was L_FA
    
    glm::quat getLFAQuat() const;
    glm::quat getRFAQuat() const;
    glm::quat getLUAQuat() const;
    glm::quat getRUAQuat() const;
    
    void calibrateLFA();   // was HIPS
    void calibrateRFA();   // was CHEST
    void calibrateLUA();
    void calibrateRUA();   // was L_FA
    void toggleQuaternionConvention();
    
    glm::quat getCorrectedLFAQuat() const;
    glm::quat getCorrectedRFAQuat() const;
    glm::quat getCorrectedLUAQuat() const;
    glm::quat getCorrectedRUAQuat() const;

    void setLTHQuat(const glm::quat& q);
    void setLSHQuat(const glm::quat& q);
    void setRTHQuat(const glm::quat& q);
    void setRSHQuat(const glm::quat& q);

    glm::quat getLTHQuat() const;
    glm::quat getLSHQuat() const;
    glm::quat getRTHQuat() const;
    glm::quat getRSHQuat() const;

    void calibrateLTH();
    void calibrateLSH();
    void calibrateRTH();
    void calibrateRSH();

    glm::quat getCorrectedLTHQuat() const;
    glm::quat getCorrectedLSHQuat() const;
    glm::quat getCorrectedRTHQuat() const;
    glm::quat getCorrectedRSHQuat() const;

private:
    static constexpr float kSmoothingAlpha = 0.35f;
    static constexpr float kFastSmoothingAlpha = 0.70f;
    static constexpr float kDeadbandRadians = 0.0009f;
    static constexpr float kFastMotionRadians = 0.20f;

    std::atomic<int> quaternionMode;

    // L_FA (was HIPS)
    mutable std::mutex quatMutex1;
    glm::quat sensorQuatLFA;
    mutable std::mutex calibMutex1;
    glm::quat calibrationReferenceLFA;
    mutable glm::quat smoothedCorrectedLFA;
    mutable bool hasSmoothedCorrectedLFA;
    
    // R_FA (was CHEST)
    mutable std::mutex quatMutex2;
    glm::quat sensorQuatRFA;
    mutable std::mutex calibMutex2;
    glm::quat calibrationReferenceRFA;
    mutable glm::quat smoothedCorrectedRFA;
    mutable bool hasSmoothedCorrectedRFA;

    // L_UA (left upper arm)
    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    mutable std::mutex calibMutex3;
    glm::quat calibrationReference3;
    mutable glm::quat smoothedCorrected3;
    mutable bool hasSmoothedCorrected3;

    // R_UA (was L_FA)
    mutable std::mutex quatMutex4;
    glm::quat sensorQuatRUA;
    mutable std::mutex calibMutex4;
    glm::quat calibrationReferenceRUA;
    mutable glm::quat smoothedCorrectedRUA;
    mutable bool hasSmoothedCorrectedRUA;

    // L_TH (left thigh)
    mutable std::mutex quatMutex5;
    glm::quat sensorQuat5;
    mutable std::mutex calibMutex5;
    glm::quat calibrationReference5;
    mutable glm::quat smoothedCorrected5;
    mutable bool hasSmoothedCorrected5;

    // L_SH (left shin)
    mutable std::mutex quatMutex6;
    glm::quat sensorQuat6;
    mutable std::mutex calibMutex6;
    glm::quat calibrationReference6;
    mutable glm::quat smoothedCorrected6;
    mutable bool hasSmoothedCorrected6;

    // R_TH (right thigh)
    mutable std::mutex quatMutex7;
    glm::quat sensorQuat7;
    mutable std::mutex calibMutex7;
    glm::quat calibrationReference7;
    mutable glm::quat smoothedCorrected7;
    mutable bool hasSmoothedCorrected7;

    // R_SH (right shin)
    mutable std::mutex quatMutex8;
    glm::quat sensorQuat8;
    mutable std::mutex calibMutex8;
    glm::quat calibrationReference8;
    mutable glm::quat smoothedCorrected8;
    mutable bool hasSmoothedCorrected8;

    void updateSensorQuat(glm::quat& current, const glm::quat& incoming) const;
    glm::quat neutralPose() const;
    glm::quat computeMotionDelta(const glm::quat& sensorQuat,
                                 const glm::quat& calibrationReference) const;
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat,
                                   const glm::quat& calibrationReference) const;
    glm::quat smoothCorrectedQuat(glm::quat& current,
                                  bool& initialized,
                                  const glm::quat& target) const;
};