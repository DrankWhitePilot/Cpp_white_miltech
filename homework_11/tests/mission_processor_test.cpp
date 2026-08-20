#include <cstring>
#include <iostream>

#include "drone_state.hpp"
#include "mission_processor.hpp"
#include "table_solver.hpp"

#ifndef HW11_TEST_DATA_FILE
#define HW11_TEST_DATA_FILE "data/ballistic_table.txt"
#endif

namespace
{
int fail(const char* message)
{
    std::cerr << "MISSION_PROCESSOR_TEST_FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    dlink::DroneCfg wireConfig{};
    wireConfig.attackSpeed = 25.0f;
    wireConfig.accelerationPath = 50.0f;
    wireConfig.angularSpeed = 0.5f;
    wireConfig.turnThreshold = 0.05f;
    wireConfig.timeStep = 0.1f;
    wireConfig.timeScale = 1.0f;

    dlink::AmmoCfg wireAmmo{};
    std::memcpy(wireAmmo.name, "M67", 3);
    wireAmmo.mass = 0.6f;
    wireAmmo.drag = 0.005f;
    wireAmmo.lift = 0.0f;
    wireAmmo.hitRadius = 2.0f;
    wireAmmo.nTargets = 1;

    DroneConfig modelConfig{};
    modelConfig.altitude = 100.0;
    modelConfig.attackSpeed = wireConfig.attackSpeed;
    modelConfig.accelPath = wireConfig.accelerationPath;
    modelConfig.arrayTimeStep = wireConfig.timeStep;
    modelConfig.simTimeStep = wireConfig.timeStep;
    modelConfig.physicsTimeStep = wireConfig.timeStep;
    modelConfig.hitRadius = wireAmmo.hitRadius;
    modelConfig.angularSpeed = wireConfig.angularSpeed;
    modelConfig.turnThreshold = wireConfig.turnThreshold;

    AmmoParams modelAmmo{};
    modelAmmo.name = "M67";
    modelAmmo.mass = wireAmmo.mass;
    modelAmmo.drag = wireAmmo.drag;
    modelAmmo.lift = wireAmmo.lift;

    TableSolver solver(HW11_TEST_DATA_FILE);
    BallisticResult plannedBallistic{};
    if (!solver.isLoaded() ||
        !solver.solve(modelConfig, modelAmmo, plannedBallistic))
    {
        return fail("ballistic table");
    }

    MissionProcessor mission;
    mission.updateConfig(wireConfig);
    mission.updateAmmo(wireAmmo);

    dlink::TargetPos target{};
    target.id = 0;
    target.x = static_cast<float>(plannedBallistic.horizontalDistance);
    target.y = 0.0f;
    mission.updateTarget(target);
    mission.updateTarget(target);

    dlink::Telemetry telemetry{};
    telemetry.t_ms = 100;
    telemetry.x = 0.0f;
    telemetry.y = 0.0f;
    telemetry.z = 100.0f;
    telemetry.speed = 2.5f;
    telemetry.dir = 0.0f;
    telemetry.state = state_code::ACCELERATING;
    mission.updateTelemetry(telemetry);

    const MissionDecision slowDecision = mission.decide();
    if (!slowDecision.valid) {
        return fail("valid slow-speed guidance");
    }
    if (slowDecision.drop) {
        return fail("DROP must use actual speed");
    }

    telemetry.t_ms = 200;
    telemetry.speed = wireConfig.attackSpeed;
    telemetry.state = state_code::MOVING;
    mission.updateTelemetry(telemetry);

    const MissionDecision attackDecision = mission.decide();
    if (!attackDecision.drop) {
        return fail("DROP at matching attack-speed impact point");
    }

    std::cout << "MISSION_PROCESSOR_TEST_PASS\n";
    return 0;
}
