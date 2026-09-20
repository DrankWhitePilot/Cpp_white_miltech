#include <chrono>
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
        telemetry.x = speed * static_cast<float>(timeMs) / 1000.0f;
        telemetry.y = 0.0f;
        telemetry.z = 100.0f;
        telemetry.vx = speed;
        telemetry.vy = 0.0f;
        telemetry.speed = speed;
        telemetry.dir = 0.0f;

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
