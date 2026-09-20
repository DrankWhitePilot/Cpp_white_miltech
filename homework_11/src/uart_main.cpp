#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "drone_controller.hpp"
#include "drone_link.h"
#include "gpio_controller.hpp"
#include "mission_processor.hpp"
#include "uart_link.hpp"

namespace
{
struct Args
{
    std::string uart = "/tmp/ttyA";
    std::string gpiochip = "gpiochip0";
    int startLine = 24;
    int dropLine = 23;
};

bool parseArgs(int argc, char* argv[], Args& args)
{
    try {
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
    } catch (const std::exception& error) {
        std::cerr << "Invalid numeric argument: " << error.what() << "\n";
        return false;
    }

    if (args.startLine < 0 ||
        args.dropLine < 0 ||
        args.startLine == args.dropLine)
    {
        std::cerr << "GPIO line numbers must be nonnegative and different\n";
        return false;
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
        std::cerr << "Usage: mission_uart_drop --uart /tmp/ttyA "
                  << "--gpiochip gpiochip0 --start-line 24 --drop-line 23\n";
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

    MissionProcessor missionProcessor;
    DroneController droneController;

    bool running = true;
    while (running) {
        UartPacket packet{};
        const UartReadResult readResult = uart.readPacket(packet);

        if (readResult == UartReadResult::WouldBlock) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (readResult == UartReadResult::Closed) {
            std::cerr << "UART connection closed\n";
            return 1;
        }

        if (readResult == UartReadResult::Error) {
            return 1;
        }

        if (packet.type == dlink::PKT_TELEMETRY) {
            dlink::Telemetry telemetry{};
            if (copyPayload(packet, telemetry)) {
                missionProcessor.updateTelemetry(telemetry);
                droneController.updateTelemetry(telemetry);

                const MissionDecision mission = missionProcessor.decide();
                const ControlCommand control =
                    droneController.control(mission);

                if (mission.drop) {
                    const bool dropHighOk = gpio.setDrop(true);
                    const bool controlOk = uart.sendControl(0.0f, 0.0f);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(100));
                    errno = 0;
                    const bool dropLowOk = gpio.setDrop(false);
                    const int dropLowErrno = errno;

                    if (!dropHighOk) {
                        std::cerr << "Cannot set DROP=1\n";
                        return 1;
                    }

                    if (!controlOk) {
                        std::cerr << "Cannot send CONTROL\n";
                        return 1;
                    }

                    if (!dropLowOk &&
                        dropLowErrno != ENODEV)
                    {
                        std::cerr << "Cannot set DROP=0\n";
                        return 1;
                    }
                } else if (!uart.sendControl(
                               control.accel,
                               control.turnRate))
                {
                    std::cerr << "Cannot send CONTROL\n";
                    return 1;
                }
            }
        } else if (packet.type == dlink::PKT_TARGET) {
            dlink::TargetPos target{};
            if (copyPayload(packet, target)) {
                missionProcessor.updateTarget(target);
            }
        } else if (packet.type == dlink::PKT_AMMO) {
            dlink::AmmoCfg ammo{};
            if (copyPayload(packet, ammo)) {
                missionProcessor.updateAmmo(ammo);
            }
        } else if (packet.type == dlink::PKT_CONFIG) {
            dlink::DroneCfg config{};
            if (copyPayload(packet, config)) {
                missionProcessor.updateConfig(config);
                droneController.updateConfig(config);
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

    return 0;
}
