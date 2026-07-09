#pragma once

#include "interfaces.hpp"

class AnalyticalSolver : public IBallisticSolver {
public:
  bool solve(const DroneConfig& config,
             const AmmoParams& ammo,
             BallisticResult& result) override;
};
