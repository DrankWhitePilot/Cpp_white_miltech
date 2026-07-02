#include "analytical_solver.hpp"

#include <cmath>

#include "model_math.hpp"

bool AnalyticalSolver::solve(
    const DroneConfig& config,
    const AmmoParams& ammo,
    const Coord* targetPath,
    int timeSteps,
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& result)
{
    if (targetPath == nullptr || timeSteps <= 0)
    {
        return false;
    }

    double fallTime = model::solveFallTime(
        ammo,
        config.altitude,
        config.attackSpeed);
    double horizontalDistance = model::calcHorizontalDistance(
        ammo,
        fallTime,
        config.attackSpeed);

    result = model::buildAttackPlanWithBallistics(
        config,
        targetPath,
        timeSteps,
        targetIndex,
        drone,
        currentTime,
        fallTime,
        horizontalDistance);

    return result.fallTime > model::EPS &&
           std::isfinite(result.horizontalDistance) &&
           result.horizontalDistance >= 0.0;
}
