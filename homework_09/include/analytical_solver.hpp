#pragma once

#include "interfaces.hpp"

class AnalyticalSolver : public IBallisticSolver {
public:
  bool solve(const DroneConfig& config,
             const AmmoParams& ammo,
             const Coord* targetPath,
             int timeSteps,
             int targetIndex,
             const DroneRuntime& drone,
             double currentTime,
             AttackPlan& result) override;
};
