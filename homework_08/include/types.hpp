#pragma once

#include <cmath>
#include <string>

enum class DroneState
{
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

enum class AttackPhase
{
    PURSUIT = 0,
    TO_MANEUVER = 1,
    ALIGN_ATTACK = 2,
    ATTACK_RUN = 3
};

struct Coord
{
    double x;
    double y;

    Coord operator+(const Coord& other) const
    {
        return {x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const
    {
        return {x - other.x, y - other.y};
    }

    Coord operator*(double factor) const
    {
        return {x * factor, y * factor};
    }

    Coord operator/(double divisor) const
    {
        return {x / divisor, y / divisor};
    }

    bool operator==(const Coord& other) const
    {
        constexpr double epsilon = 1e-9;
        return std::fabs(x - other.x) <= epsilon &&
               std::fabs(y - other.y) <= epsilon;
    }
};

struct DroneConfig
{
    Coord startPos;
    double altitude;
    double initialDir;
    double attackSpeed;
    double accelPath;
    std::string ammoName;
    double arrayTimeStep;
    double simTimeStep;
    double hitRadius;
    double angularSpeed;
    double turnThreshold;
};

struct AmmoParams
{
    std::string name;
    double mass;
    double drag;
    double lift;
};

struct SimStep
{
    Coord position;
    double direction;
    DroneState state;
    int targetIndex;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

struct DroneRuntime
{
    Coord position;
    double direction;
    double speed;
    DroneState state;
};

struct DropPlan
{
    bool needManeuver;
    Coord maneuverPoint;
    Coord firePoint;
};

struct AttackPlan
{
    int targetIndex;
    Coord targetNow;
    Coord targetVelocity;
    Coord predictedTarget;
    Coord impactTarget;
    double predictionUncertainty;
    DropPlan dropPlan;
    double fallTime;
    double horizontalDistance;
    double timeToDrop;
    double totalTime;
};

struct MissionRuntime
{
    int targetIndex;
    AttackPhase phase;
    Coord maneuverPoint;
    Coord firePoint;
    Coord impactTarget;
    double attackDirection;
    double horizontalDistance;
};
