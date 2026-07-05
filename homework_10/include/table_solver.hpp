#pragma once

#include <string>

#include "ballistic_table.hpp"
#include "i_ballistic_solver.hpp"

class TableSolver final : public IBallisticSolver
{
public:
    explicit TableSolver(std::string tablePath);

    bool solve(const DroneConfig& config,
               const AmmoParams& ammo,
               BallisticResult& result) override;

    bool isLoaded() const;

private:
    BallisticTable table_;
};
