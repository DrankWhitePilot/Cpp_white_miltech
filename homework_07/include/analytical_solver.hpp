#pragma once

#include "interfaces.hpp"

class AnalyticalSolver : public IBallisticSolver
{
public:
    bool solve(
        InputData& data,
        double droneX,
        double droneY,
        double targetX,
        double targetY,
        double& aimX,
        double& aimY,
        double& fireX,
        double& fireY,
        double& totalTime,
        bool& needManeuver) override;

    bool solve(
        const DroneConfig& config,
        const AmmoParams& ammo,
        const Coord* targetPath,
        int timeSteps,
        int targetIndex,
        const DroneRuntime& drone,
        double currentTime,
        AttackPlan& result) override;
};
