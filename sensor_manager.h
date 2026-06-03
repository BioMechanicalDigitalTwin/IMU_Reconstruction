#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <mutex>

class SensorManager {
public:
    SensorManager();
    
    void setHipsQuat(const glm::quat& q);
    void setChestQuat(const glm::quat& q);
    
    glm::quat getHipsQuat() const;
    glm::quat getChestQuat() const;
    
    void calibrateHips();
    void calibrateChest();
    
    glm::quat getCorrectedHipsQuat() const;
    glm::quat getCorrectedChestQuat() const;

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
    
    const glm::quat sensorOffset;
    
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat, 
                                   const glm::quat& calibOffset) const;
};