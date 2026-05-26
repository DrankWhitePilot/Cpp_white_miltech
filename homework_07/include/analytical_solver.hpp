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
};
