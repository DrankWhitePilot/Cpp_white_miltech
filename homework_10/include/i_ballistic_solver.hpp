#pragma once

#include "types.hpp"

class IBallisticSolver
{
public:
    virtual ~IBallisticSolver() = default;

    virtual bool solve(const DroneConfig& config,
                       const AmmoParams& ammo,
                       BallisticResult& result) = 0;
};
