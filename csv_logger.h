#pragma once
#include <string>
#include <fstream>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class CsvLogger {
public:
    CsvLogger();
    ~CsvLogger();

    // Call once after OpenGL init, before render loop
    bool open();

    // Call every frame with the 10 corrected quats
    void log(const glm::quat& lfa, const glm::quat& rfa,
             const glm::quat& lua, const glm::quat& rua,
             const glm::quat& lth, const glm::quat& lsh,
             const glm::quat& rth, const glm::quat& rsh,
             const glm::quat& hips, const glm::quat& chest);

    void close();

private:
    std::ofstream file;
    std::chrono::steady_clock::time_point startTime;
    int frameCount = 0;
    static constexpr int kFlushEveryN = 60; // flush ~every second at 60Hz

    static std::string makeFilepath();
    static bool ensureDir(const std::string& dir);
};