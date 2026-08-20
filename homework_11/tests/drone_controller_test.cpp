#include <cmath>
#include <iostream>

#include "drone_controller.hpp"

namespace
{
bool near(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) <= 1e-6f;
}

int fail(const char* message)
{
    std::cerr << "DRONE_CONTROLLER_TEST_FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    DroneController controller;
    MissionDecision mission{};

    ControlCommand command = controller.control(mission);
    if (!near(command.accel, 0.0f) || !near(command.turnRate, 0.0f)) {
        return fail("uninitialized controller must be neutral");
    }

    dlink::DroneCfg config{};
    config.attackSpeed = 10.0f;
    config.accelerationPath = 20.0f;
    config.angularSpeed = 1.0f;
    config.turnThreshold = 0.05f;
    config.timeStep = 0.1f;
    config.timeScale = 1.0f;
    controller.updateConfig(config);

    dlink::Telemetry telemetry{};
    telemetry.x = 0.0f;
    telemetry.y = 0.0f;
    telemetry.dir = 0.0f;
    telemetry.speed = 5.0f;
    controller.updateTelemetry(telemetry);

    mission.valid = true;
    mission.destination = {100.0, 0.0};
    mission.desiredSpeed = 10.0;
    command = controller.control(mission);
    if (!near(command.accel, 1.0f) || !near(command.turnRate, 0.0f)) {
        return fail("accelerate toward aligned destination");
    }

    mission.destination = {0.0, 100.0};
    command = controller.control(mission);
    if (!near(command.accel, -1.0f) || !near(command.turnRate, 0.0f)) {
        return fail("decelerate before turn");
    }

    telemetry.speed = 0.0f;
    controller.updateTelemetry(telemetry);
    command = controller.control(mission);
    if (!near(command.accel, 0.0f) || !near(command.turnRate, 1.0f)) {
        return fail("turn in place with normalized command");
    }

    if (std::fabs(command.accel) > 1.0f ||
        std::fabs(command.turnRate) > 1.0f)
    {
        return fail("command range");
    }

    std::cout << "DRONE_CONTROLLER_TEST_PASS\n";
    return 0;
}
