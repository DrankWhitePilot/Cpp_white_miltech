#pragma once

struct Coord {
    double x;
    double y;

    Coord operator+(const Coord& other) const {
        return {x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const {
        return {x - other.x, y - other.y};
    }

    Coord operator*(double s) const {
        return {x * s, y * s};
    }

    Coord operator/(double s) const {
        return {x / s, y / s};
    }

    bool operator==(const Coord& other) const {
        return (x == other.x) && (y == other.y);
    }
};

struct DroneConfig {
    Coord startPos;
    double altitude;
    double initialDir;
    double attackSpeed;
    double accelPath;
    char ammoName[32];
    double arrayTimeStep;
    double simTimeStep;
    double hitRadius;
    double angularSpeed;
    double turnThreshold;
};

struct AmmoParams {
    char name[32];
    double mass;
    double drag;
    double lift;
};

struct SimStep
{
    Coord pos;
    double direction;
    int state;
    int targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

struct InputData
{
    double xd, yd, zd;
    double initialDir;
    double attackSpeed;
    double accelerationPath;
    char ammo_name[32];
    double arrayTimeStep;
    double simTimeStep;
    double hitRadius;
    double angularSpeed;
    double turnThreshold;
    double m;
    double d;
    double l;
    double targetX;
    double targetY;
};

enum DroneState
{
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

struct DropPoint
{
    bool valid;
    int targetIdx;
    Coord point;
    Coord aimPoint;
    double totalTime;
    bool needManeuver;
};

enum class AttackPhase
{
    PURSUIT = 0,
    TO_MANEUVER = 1,
    ALIGN_ATTACK = 2,
    ATTACK_RUN = 3
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
