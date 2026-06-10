#include "csv_logger.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>

CsvLogger::CsvLogger() = default;

CsvLogger::~CsvLogger()
{
    close();
}

bool CsvLogger::ensureDir(const std::string& dir)
{
    struct stat st{};
    if (stat(dir.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(dir.c_str(), 0755) == 0;
}

std::string CsvLogger::makeFilepath()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);

    return std::string("CSV/") + buf + ".csv";
}

bool CsvLogger::open()
{
    if (!ensureDir("CSV")) {
        std::cerr << "[CsvLogger] Failed to create CSV/ directory\n";
        return false;
    }

    std::string path = makeFilepath();
    file.open(path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "[CsvLogger] Failed to open: " << path << "\n";
        return false;
    }

    // Header
    file << "time,"
         << "L_FA_w,L_FA_x,L_FA_y,L_FA_z,"
         << "R_FA_w,R_FA_x,R_FA_y,R_FA_z,"
         << "L_UA_w,L_UA_x,L_UA_y,L_UA_z,"
         << "R_UA_w,R_UA_x,R_UA_y,R_UA_z,"
         << "L_TH_w,L_TH_x,L_TH_y,L_TH_z,"
         << "L_SH_w,L_SH_x,L_SH_y,L_SH_z,"
         << "R_TH_w,R_TH_x,R_TH_y,R_TH_z,"
         << "R_SH_w,R_SH_x,R_SH_y,R_SH_z,"
         << "HIPS_w,HIPS_x,HIPS_y,HIPS_z,"
         << "CHEST_w,CHEST_x,CHEST_y,CHEST_z\n";

    startTime = std::chrono::steady_clock::now();
    std::cout << "[CsvLogger] Logging to: " << path << "\n";
    return true;
}

static void writeQuat(std::ostream& os, const glm::quat& q, bool last = false)
{
    os << q.w << ',' << q.x << ',' << q.y << ',' << q.z;
    if (!last) os << ',';
}

void CsvLogger::log(const glm::quat& lfa, const glm::quat& rfa,
                    const glm::quat& lua, const glm::quat& rua,
                    const glm::quat& lth, const glm::quat& lsh,
                    const glm::quat& rth, const glm::quat& rsh,
                    const glm::quat& hips, const glm::quat& chest)
{
    if (!file.is_open()) return;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();

    file << std::fixed << std::setprecision(6) << elapsed << ',';
    writeQuat(file, lfa);
    writeQuat(file, rfa);
    writeQuat(file, lua);
    writeQuat(file, rua);
    writeQuat(file, lth);
    writeQuat(file, lsh);
    writeQuat(file, rth);
    writeQuat(file, rsh);
    writeQuat(file, hips);
    writeQuat(file, chest, /*last=*/true);
    file << '\n';

    if (++frameCount % kFlushEveryN == 0)
        file.flush();
}

void CsvLogger::close()
{
    if (file.is_open()) {
        file.flush();
        file.close();
        std::cout << "[CsvLogger] File closed.\n";
    }
}