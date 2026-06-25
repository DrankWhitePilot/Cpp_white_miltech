#pragma once

#include <string>

#include "ballistic_table.hpp"
#include "interfaces.hpp"

class TableSolver final : public IBallisticSolver
{
public:
    explicit TableSolver(std::string tablePath);

    bool solve(const DroneConfig& config,
               const AmmoParams& ammo,
               const Coord* targetPath,
               int timeSteps,
               int targetIndex,
               const DroneRuntime& drone,
               double currentTime,
               AttackPlan& result) override;

    bool isLoaded() const;

private:
    BallisticTable table_;
};
