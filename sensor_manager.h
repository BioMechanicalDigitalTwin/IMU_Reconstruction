#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
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
    
    glm::quat getCorrectedHipsQuat() const;
    glm::quat getCorrectedChestQuat() const;
    glm::quat getCorrectedLUAQuat() const;
    glm::quat getCorrectedLFAQuat() const;

private:
    // HIPS (hand 1)
    mutable std::mutex quatMutex1;
    glm::quat sensorQuat1;
    mutable std::mutex calibMutex1;
    glm::quat calibrationOffset1;
    
    // CHEST (hand 2)
    mutable std::mutex quatMutex2;
    glm::quat sensorQuat2;
    mutable std::mutex calibMutex2;
    glm::quat calibrationOffset2;

    // L_UA (left upper arm)
    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    mutable std::mutex calibMutex3;
    glm::quat calibrationOffset3;

    // L_FA (right upper arm)
    mutable std::mutex quatMutex4;
    glm::quat sensorQuat4;
    mutable std::mutex calibMutex4;
    glm::quat calibrationOffset4;
    
    const glm::quat sensorOffset;
    
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat, 
                                   const glm::quat& calibOffset) const;
};