#pragma once

#include "interfaces.hpp"

class AnalyticalSolver : public IBallisticSolver
{
public:
    bool solve(const DroneConfig& config,
               const AmmoParams& ammo,
               const Target& target,
               int targetIndex,
               const DroneRuntime& drone,
               double currentTime,
               AttackPlan& result) override;
};
