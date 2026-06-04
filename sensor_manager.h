#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <mutex>

class SensorManager {
public:
    SensorManager();
    
    void setHipsQuat(const glm::quat& q);
    void setChestQuat(const glm::quat& q);
    void setLUAQuat(const glm::quat& q);
    void setLFAQuat(const glm::quat& q);
    
    glm::quat getHipsQuat() const;
    glm::quat getChestQuat() const;
    glm::quat getLUAQuat() const;
    glm::quat getLFAQuat() const;
    
    void calibrateHips();
    void calibrateChest();
    void calibrateLUA();
    void calibrateLFA();
    void toggleQuaternionConvention();
    
    glm::quat getCorrectedHipsQuat() const;
    glm::quat getCorrectedChestQuat() const;
    glm::quat getCorrectedLUAQuat() const;
    glm::quat getCorrectedLFAQuat() const;

private:
    static constexpr float kSmoothingAlpha = 0.35f;
    static constexpr float kFastSmoothingAlpha = 0.70f;
    static constexpr float kDeadbandRadians = 0.0009f;
    static constexpr float kFastMotionRadians = 0.20f;

    std::atomic<int> quaternionMode;

    // HIPS (hand 1)
    mutable std::mutex quatMutex1;
    glm::quat sensorQuat1;
    mutable std::mutex calibMutex1;
    glm::quat calibrationReference1;
    mutable glm::quat smoothedCorrected1;
    mutable bool hasSmoothedCorrected1;
    
    // CHEST (hand 2)
    mutable std::mutex quatMutex2;
    glm::quat sensorQuat2;
    mutable std::mutex calibMutex2;
    glm::quat calibrationReference2;
    mutable glm::quat smoothedCorrected2;
    mutable bool hasSmoothedCorrected2;

    // L_UA (left upper arm)
    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    mutable std::mutex calibMutex3;
    glm::quat calibrationReference3;
    mutable glm::quat smoothedCorrected3;
    mutable bool hasSmoothedCorrected3;

    // L_FA (right upper arm)
    mutable std::mutex quatMutex4;
    glm::quat sensorQuat4;
    mutable std::mutex calibMutex4;
    glm::quat calibrationReference4;
    mutable glm::quat smoothedCorrected4;
    mutable bool hasSmoothedCorrected4;

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
