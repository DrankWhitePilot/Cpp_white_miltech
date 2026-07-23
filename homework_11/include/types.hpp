#pragma once

#include <string>

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

    Coord operator+(const Coord& other) const;
    Coord operator-(const Coord& other) const;
    Coord operator*(double factor) const;
    Coord operator/(double divisor) const;
    bool operator==(const Coord& other) const;
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
    double physicsTimeStep;
    double timeScale;
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

struct BallisticResult
{
    double fallTime;
    double horizontalDistance;
};

struct SimStep
{
    Coord position;
    double direction;
    int state;
    int targetIndex;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
    double timeSecSinceStart;
};

struct Target
{
    Coord pos;
    Coord velocity;
};

struct DroneTelemetry
{
    Coord pos;
    Coord speed;
    double direction;
    int state;
    double timeSecSinceStart;
};

struct DroneRuntime
{
    Coord position;
    double direction;
    double speed;
    int state;
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
