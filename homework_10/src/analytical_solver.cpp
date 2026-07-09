#include "analytical_solver.hpp"

#include <cmath>

#include "model_math.hpp"

bool AnalyticalSolver::solve(
    const DroneConfig& config,
    const AmmoParams& ammo,
    BallisticResult& result)
{
    result.fallTime = model::solveFallTime(
        ammo,
        config.altitude,
        config.attackSpeed);
    result.horizontalDistance = model::calcHorizontalDistance(
        ammo,
        result.fallTime,
        config.attackSpeed);

    return result.fallTime > model::EPS &&
           std::isfinite(result.horizontalDistance) &&
           result.horizontalDistance >= 0.0;
}
