#pragma once

#include "i_ballistic_solver.hpp"

class AnalyticalSolver final : public IBallisticSolver
{
public:
    bool solve(const DroneConfig& config,
               const AmmoParams& ammo,
               BallisticResult& result) override;
};
