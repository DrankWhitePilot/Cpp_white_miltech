#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "mavlink_udp.hpp"

int main()
{
    MavlinkUdp link("127.0.0.1", 14550);
    if (!link.openSocket()) {
        return 1;
    }

    constexpr uint32_t stepMs = 100;
    constexpr float speed = 10.0f;
    bool dropStarted = false;

    for (uint32_t timeMs = 0; timeMs <= 7000; timeMs += stepMs) {
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
