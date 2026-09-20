#pragma once

#include "types.hpp"

class IBallisticSolver {
public:
  virtual bool solve(const DroneConfig& config,
                     const AmmoParams& ammo,
                     const Coord* targetPath,
                     int timeSteps,
                     int targetIndex,
                     const DroneRuntime& drone,
                     double currentTime,
                     AttackPlan& result) = 0;

  virtual ~IBallisticSolver() = default;
};
