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
    double hitRadius;
    double angularSpeed;
    double turnThreshold;
    double targetTimeStep = 0.05;
    double physicsTimeStep = 0.01;
    double timeScale = 10.0;
};

struct Target
{
    Coord pos;
    Coord velocity;
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
    double fallTime = 0.0;
    double horizontalDistance = 0.0;
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
    double timeSecSinceStart = 0.0;
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
