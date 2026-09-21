#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "telemetry_log.hpp"

static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char* argv[])
{
    if (argc != 2) return 2;
    const std::string path = argv[1];
    try {
        TelemetryLog disabled;
        require(disabled.open(""), "disabled logger");
        require(disabled.append({}), "disabled append");
        TelemetryLog log;
        require(log.open(path), "new log must open");
        TelemetryLog duplicate;
        require(!duplicate.open(path), "existing file must be protected");
        dlink::Telemetry sample{};
        sample.t_ms = 1000;
        sample.x = 1.25F;
        sample.y = -2.5F;
        sample.z = 10.0F;
        sample.vx = 3.0F;
        sample.vy = 4.0F;
        sample.speed = 5.0F;
        sample.dir = 0.5F;
        sample.state = 1;
        require(log.append(sample), "first append");
        sample.x = std::numeric_limits<float>::quiet_NaN();
        require(!log.append(sample), "reject non-finite input");
        sample.t_ms = 1100;
        sample.x = 1.55F;
        require(log.append(sample), "valid sample after invalid input");
        std::ifstream file(path);
        std::stringstream content;
        content << file.rdbuf();
        const auto text = content.str();
        require(text.find("# hw11-telemetry-v1\nt_ms,x,y,z,vx,vy,speed,dir,state\n") == 0,
                "schema header");
        require(text.find("1000,1.25,-2.5,10,3,4,5,0.5,1\n") != std::string::npos,
                "field order and units");
        require(text.find("nan") == std::string::npos, "no invalid row");
        std::cout << "PASS: telemetry logger; synthetic fixture " << path << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
