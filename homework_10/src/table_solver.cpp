#include "table_solver.hpp"

#include <cmath>
#include <utility>

#include "model_math.hpp"

TableSolver::TableSolver(std::string tablePath)
{
    table_.load(tablePath);
}

bool TableSolver::isLoaded() const
{
    return table_.valid();
}

bool TableSolver::solve(
    const DroneConfig& config,
    const AmmoParams& ammo,
    const Target& target,
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& result)
{
    (void)currentTime;
    if (!table_.valid())
    {
        return false;
    }

    BallisticTable::Result tableResult = table_.lookup(
        config.altitude,
        config.attackSpeed,
        ammo.mass,
        ammo.drag,
        ammo.lift);

    if (!std::isfinite(tableResult.t) ||
        !std::isfinite(tableResult.hDist) ||
        tableResult.t <= model::EPS ||
        tableResult.hDist < 0.0)
    {
        return false;
    }

    result = model::buildAttackPlanWithBallistics(
        config,
        target,
        targetIndex,
        drone,
        tableResult.t,
        tableResult.hDist);
    return true;
}
