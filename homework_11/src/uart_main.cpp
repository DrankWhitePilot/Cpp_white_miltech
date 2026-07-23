#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "drone_controller.hpp"
#include "drone_link.h"
#include "gpio_controller.hpp"
#include "uart_link.hpp"

namespace
{
struct Args
{
    std::string uart = "/tmp/ttyA";
    std::string gpiochip = "gpiochip1";
    int startLine = 24;
    int dropLine = 23;
};

bool parseArgs(int argc, char* argv[], Args& args)
{
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];

        if (key == "--uart" && i + 1 < argc) {
            args.uart = argv[++i];
        } else if (key == "--gpiochip" && i + 1 < argc) {
            args.gpiochip = argv[++i];
        } else if (key == "--start-line" && i + 1 < argc) {
            args.startLine = std::stoi(argv[++i]);
        } else if (key == "--drop-line" && i + 1 < argc) {
            args.dropLine = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete argument: " << key << "\n";
            return false;
        }
    }

    return true;
}

template <typename T>
bool copyPayload(const UartPacket& packet, T& value)
{
    if (packet.len != sizeof(T)) {
        return false;
    }

    std::memcpy(&value, packet.payload, sizeof(T));
    return true;
}
}

int main(int argc, char* argv[])
{
    Args args;
    if (!parseArgs(argc, argv, args)) {
        std::cerr << "Usage: mission_uart_drop --uart /tmp/ttyA --gpiochip gpiochipN "
                  << "--start-line 24 --drop-line 23\n";
        return 1;
    }

    UartLink uart(args.uart);
    if (!uart.openPort()) {
        return 1;
    }

    GpioController gpio;
    if (!gpio.init(args.gpiochip, args.startLine, args.dropLine)) {
        return 1;
    }

    if (!gpio.setStart(true)) {
        std::cerr << "Cannot set START=1\n";
        return 1;
    }

    DroneController controller;

    bool running = true;
    while (running) {
        UartPacket packet{};
        bool gotPacket = false;

        while (uart.readPacket(packet)) {
            gotPacket = true;

            if (packet.type == dlink::PKT_TELEMETRY) {
                dlink::Telemetry telemetry{};
                if (copyPayload(packet, telemetry)) {
                    controller.updateTelemetry(telemetry);

                    const ControlDecision decision = controller.decide();
                    uart.sendControl(decision.accel, decision.turnRate);

                    if (decision.drop) {
                        gpio.pulseDrop(80000);
                        std::cout << "DROP target=" << decision.targetIndex
                                  << " aim=(" << decision.aimPoint.x << ", "
                                  << decision.aimPoint.y << ") predicted=("
                                  << decision.predictedTarget.x << ", "
                                  << decision.predictedTarget.y << ")\n";
                    }
                }
            } else if (packet.type == dlink::PKT_TARGET) {
                dlink::TargetPos target{};
                if (copyPayload(packet, target)) {
                    controller.updateTarget(target);
                }
            } else if (packet.type == dlink::PKT_AMMO) {
                dlink::AmmoCfg ammo{};
                if (copyPayload(packet, ammo)) {
                    controller.updateAmmo(ammo);
                    std::cout << "AMMO name=" << ammo.name
                              << " targets=" << static_cast<int>(ammo.nTargets)
                              << " hitRadius=" << ammo.hitRadius << "\n";
                }
            } else if (packet.type == dlink::PKT_CONFIG) {
                dlink::DroneCfg config{};
                if (copyPayload(packet, config)) {
                    controller.updateConfig(config);
                    std::cout << "CONFIG attackSpeed=" << config.attackSpeed
                              << " angularSpeed=" << config.angularSpeed
                              << " timeStep=" << config.timeStep << "\n";
                }
            } else if (packet.type == dlink::PKT_RESULT) {
                dlink::Result result{};
                if (copyPayload(packet, result)) {
                    std::cout << "RESULT hit=" << static_cast<int>(result.hit)
                              << " target=" << static_cast<int>(result.targetId)
                              << " miss=" << result.miss_m
                              << " drop_t_ms=" << result.drop_t_ms << "\n";
                    running = false;
                }
            }
        }

        if (!gotPacket) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    return 0;
}
