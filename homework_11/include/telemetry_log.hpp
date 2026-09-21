#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <string>

#include "drone_link.h"

// Optional, read-only record of received telemetry. No control decisions are logged.
class TelemetryLog
{
public:
    bool open(const std::string& path)
    {
        if (path.empty()) return true;
        std::error_code error;
        if (std::filesystem::exists(path, error) || error) return false;
        stream_.open(path, std::ios::out);
        if (!stream_) return false;
        stream_.imbue(std::locale::classic());
        stream_ << std::setprecision(std::numeric_limits<float>::max_digits10)
                << "# hw11-telemetry-v1\n"
                << "t_ms,x,y,z,vx,vy,speed,dir,state\n";
        stream_.flush();
        return stream_.good();
    }

    bool append(const dlink::Telemetry& value)
    {
        if (!stream_.is_open()) return true;
        if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
            !std::isfinite(value.z) || !std::isfinite(value.vx) ||
            !std::isfinite(value.vy) || !std::isfinite(value.speed) ||
            !std::isfinite(value.dir)) return false;
        stream_ << value.t_ms << ',' << value.x << ',' << value.y << ','
                << value.z << ',' << value.vx << ',' << value.vy << ','
                << value.speed << ',' << value.dir << ','
                << static_cast<unsigned>(value.state) << '\n';
        // Keep completed rows available after an interrupted run.
        stream_.flush();
        return stream_.good();
    }

private:
    std::ofstream stream_;
};
