#pragma once

#include "types.hpp"

namespace model
{
inline constexpr double G = 9.81;
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double TWO_PI = 2.0 * PI;
inline constexpr double EPS = 1e-9;

struct ObservedTargetState
{
    Coord position;
    Coord velocity;
    Coord acceleration;
};

double length(Coord value);
Coord normalize(Coord value);
double normalizeAngleTwoPi(double angleRadians);
double calcTurnDeltaRadians(double currentDirection, double desiredDirection);
double directionToRadians(Coord from, Coord to, double fallbackDirection);
double calcDroneAcceleration(double attackSpeed, double accelerationPath);
Coord directionVector(double directionRadians);
Coord calcAimPoint(Coord position, double directionRadians, double horizontalDistance);

ObservedTargetState observeTargetFromPast(
    const Coord* targetPath,
    int timeSteps,
    double time,
    double arrayTimeStep);

Coord predictObservedTarget(
    const ObservedTargetState& state,
    double horizon);

Coord estimatePredictionResidual(
    const Coord* targetPath,
    int timeSteps,
    double currentTime,
    double arrayTimeStep,
    double horizon);

AttackPlan buildAttackPlanWithBallistics(
    const DroneConfig& config,
    const Coord* targetPath,
    int timeSteps,
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    double fallTime,
    double horizontalDistance);

AttackPlan buildAttackPlanWithBallistics(
    const DroneConfig& config,
    const Target& target,
    int targetIndex,
    const DroneRuntime& drone,
    double fallTime,
    double horizontalDistance);

DropPlan calculateDynamicDropPlan(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord target,
    double horizontalDistance,
    double& timeToRelease);
}
