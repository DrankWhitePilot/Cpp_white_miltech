#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "mavlink_udp.hpp"

int main(int argc, char* argv[])
{
    const std::string address = argc > 1 ? argv[1] : "127.0.0.1";
    const uint32_t durationMs = argc > 2
                                    ? static_cast<uint32_t>(std::stoul(argv[2])) * 1000U
                                    : 7000U;
    const bool turningDemo = argc > 3 && std::string(argv[3]) == "turn";
    MavlinkUdp link(address, 14550);
    if (!link.openSocket()) {
        return 1;
    }

    constexpr uint32_t stepMs = 100;
    constexpr float speed = 10.0f;
    bool dropStarted = false;

    for (uint32_t timeMs = 0; timeMs <= durationMs; timeMs += stepMs) {
        dlink::Telemetry telemetry{};
        telemetry.t_ms = timeMs;
        const float timeSec = static_cast<float>(timeMs) / 1000.0f;
        if (turningDemo) {
            constexpr float turnRate = 0.08f;
            const float direction = turnRate * timeSec;
            telemetry.x = speed / turnRate * std::sin(direction);
            telemetry.y = speed / turnRate * (1.0f - std::cos(direction));
            telemetry.vx = speed * std::cos(direction);
            telemetry.vy = speed * std::sin(direction);
            telemetry.dir = direction;
        } else {
            telemetry.x = speed * timeSec;
            telemetry.vx = speed;
        }
        telemetry.z = 100.0f;
        telemetry.speed = speed;

        if (!link.sendTelemetry(telemetry)) {
            return 1;
        }
        if (!dropStarted && timeMs >= 2000) {
            if (!link.startDropCommand(telemetry)) {
                return 1;
            }
            dropStarted = true;
        }

        link.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(stepMs));
    }

    link.poll();
    std::cout << "CHECKER_DRIVER_DONE attempts=" << link.dropAttempts()
              << " acknowledged=" << link.dropAcknowledged() << '\n';
    return link.dropAcknowledged() ? 0 : 1;
}
