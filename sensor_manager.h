#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <mutex>

class SensorManager {
public:
    SensorManager();
    
    void setLFAQuat(const glm::quat& q);
    void setRFAQuat(const glm::quat& q);
    void setLUAQuat(const glm::quat& q);
    void setRUAQuat(const glm::quat& q);
    
    glm::quat getLFAQuat() const;
    glm::quat getRFAQuat() const;
    glm::quat getLUAQuat() const;
    glm::quat getRUAQuat() const;
    
    void calibrateLFA();
    void calibrateRFA();
    void calibrateLUA();
    void calibrateRUA();
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

    void setHipsQuat(const glm::quat& q);
    void setChestQuat(const glm::quat& q);

    glm::quat getHipsQuat()  const;
    glm::quat getChestQuat() const;

    void calibrateHips();
    void calibrateChest();

    glm::quat getCorrectedHipsQuat()  const;
    glm::quat getCorrectedChestQuat() const;

private:
    static constexpr float kSmoothingAlpha       = 0.55f;   
    static constexpr float kFastSmoothingAlpha   = 0.80f;  
    static constexpr float kDeadbandRadians      = 0.008f;  
    static constexpr float kFastMotionRadians    = 0.20f;   
    static constexpr float kStationaryThreshold  = 0.015f;  
    static constexpr float kStationaryTimeMs     = 5000.0f; 
    static constexpr float kDriftCorrAlpha       = 0.0003f; 
    
    std::atomic<int> quaternionMode;

    mutable std::mutex quatMutex1;
    glm::quat sensorQuatLFA;
    mutable std::mutex calibMutex1;
    mutable glm::quat calibrationReferenceLFA;
    mutable glm::quat smoothedCorrectedLFA;
    mutable bool hasSmoothedCorrectedLFA;
    mutable glm::quat lastQLFA;
    mutable float stationaryTimerLFA;

    mutable std::mutex quatMutex2;
    glm::quat sensorQuatRFA;
    mutable std::mutex calibMutex2;
    mutable glm::quat calibrationReferenceRFA;
    mutable glm::quat smoothedCorrectedRFA;
    mutable bool hasSmoothedCorrectedRFA;
    mutable glm::quat lastQRFA;
    mutable float stationaryTimerRFA;

    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    mutable std::mutex calibMutex3;
    mutable glm::quat calibrationReference3;
    mutable glm::quat smoothedCorrected3;
    mutable bool hasSmoothedCorrected3;
    mutable glm::quat lastQ3;
    mutable float stationaryTimer3;

    mutable std::mutex quatMutex4;
    glm::quat sensorQuatRUA;
    mutable std::mutex calibMutex4;
    mutable glm::quat calibrationReferenceRUA;
    mutable glm::quat smoothedCorrectedRUA;
    mutable bool hasSmoothedCorrectedRUA;
    mutable glm::quat lastQRUA;
    mutable float stationaryTimerRUA;

    mutable std::mutex quatMutex5;
    glm::quat sensorQuat5;
    mutable std::mutex calibMutex5;
    mutable glm::quat calibrationReference5;
    mutable glm::quat smoothedCorrected5;
    mutable bool hasSmoothedCorrected5;
    mutable glm::quat lastQ5;
    mutable float stationaryTimer5;

    mutable std::mutex quatMutex6;
    glm::quat sensorQuat6;
    mutable std::mutex calibMutex6;
    mutable glm::quat calibrationReference6;
    mutable glm::quat smoothedCorrected6;
    mutable bool hasSmoothedCorrected6;
    mutable glm::quat lastQ6;
    mutable float stationaryTimer6;

    mutable std::mutex quatMutex7;
    glm::quat sensorQuat7;
    mutable std::mutex calibMutex7;
    mutable glm::quat calibrationReference7;
    mutable glm::quat smoothedCorrected7;
    mutable bool hasSmoothedCorrected7;
    mutable glm::quat lastQ7;
    mutable float stationaryTimer7;

    mutable std::mutex quatMutex8;
    glm::quat sensorQuat8;
    mutable std::mutex calibMutex8;
    mutable glm::quat calibrationReference8;
    mutable glm::quat smoothedCorrected8;
    mutable bool hasSmoothedCorrected8;
    mutable glm::quat lastQ8;
    mutable float stationaryTimer8;

    mutable std::mutex quatMutex9;
    glm::quat sensorQuat9;
    mutable std::mutex calibMutex9;
    mutable glm::quat calibrationReference9;
    mutable glm::quat smoothedCorrected9;
    mutable bool hasSmoothedCorrected9;
    mutable glm::quat lastQ9;
    mutable float stationaryTimer9;

    mutable std::mutex quatMutex10;
    glm::quat sensorQuat10;
    mutable std::mutex calibMutex10;
    mutable glm::quat calibrationReference10;
    mutable glm::quat smoothedCorrected10;
    mutable bool hasSmoothedCorrected10;
    mutable glm::quat lastQ10;
    mutable float stationaryTimer10;

    void updateSensorQuat(glm::quat& current, const glm::quat& incoming) const;
    glm::quat neutralPose() const;
    glm::quat computeMotionDelta(const glm::quat& sensorQuat,
                                 const glm::quat& calibrationReference) const;
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat,
                                   const glm::quat& calibrationReference) const;
    glm::quat smoothCorrectedQuat(glm::quat& current,
                                  bool& initialized,
                                  const glm::quat& target) const;
    void autoRecalibrate(glm::quat& calibRef,
                         glm::quat& lastQ,
                         float& stationaryTimer,
                         const glm::quat& current) const;
};