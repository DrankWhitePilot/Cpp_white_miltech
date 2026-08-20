#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "model_math.hpp"
#include "table_solver.hpp"

namespace {
int fail(const char* msg)
{
    std::cerr << "FAIL " << msg << "\n";
    return 1;
}

bool near(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) <= eps;
}
}

int main(int argc, char** argv)
{
    const std::string tablePath =
        argc > 1 ? argv[1] : "data/ballistic_table.txt";

    if (!near(model::length({3.0, 4.0}), 5.0)) {
        return fail("length");
    }

    if (!near(model::calcDroneAcceleration(10.0, 10.0), 5.0)) {
        return fail("calcDroneAcceleration");
    }

    if (!near(model::calcTurnDeltaRadians(0.0, model::PI / 2.0),
              model::PI / 2.0)) {
        return fail("turn_left");
    }

    if (!near(model::calcTurnDeltaRadians(0.0, -model::PI / 2.0),
              -model::PI / 2.0)) {
        return fail("turn_right");
    }

    AmmoParams ammo{"VOG-17", 0.35, 0.004, 0.0};

    DroneConfig config{};
    config.altitude = 100.0;
    config.attackSpeed = 10.0;
    config.accelPath = 10.0;
    config.arrayTimeStep = 0.1;
    config.simTimeStep = 0.1;
    config.physicsTimeStep = 0.01;
    config.timeScale = 1.0;
    config.hitRadius = 3.0;
    config.angularSpeed = 1.0;
    config.turnThreshold = 0.1;

    const double fall = model::solveFallTime(
        ammo,
        config.altitude,
        config.attackSpeed);

    const double dist = model::calcHorizontalDistance(
        ammo,
        fall,
        config.attackSpeed);

    if (!std::isfinite(fall) || fall <= 0.0) {
        return fail("solveFallTime");
    }

    if (!std::isfinite(dist) || dist < 0.0) {
        return fail("calcHorizontalDistance");
    }

    TableSolver solver(tablePath);
    if (!solver.isLoaded()) {
        return fail("table_load");
    }

    BallisticResult result{};
    if (!solver.solve(config, ammo, result)) {
        return fail("table_solve");
    }

    if (!std::isfinite(result.fallTime) || result.fallTime <= 0.0) {
        return fail("table_fallTime");
    }

    if (!std::isfinite(result.horizontalDistance) ||
        result.horizontalDistance < 0.0) {
        return fail("table_horizontalDistance");
    }

    std::cout << "MATH_PROBE_PASS\n";
    std::cout << "fallTime=" << fall
              << " horizontalDistance=" << dist << "\n";
    std::cout << "tableFallTime=" << result.fallTime
              << " tableHorizontalDistance=" << result.horizontalDistance << "\n";

    return 0;
}
