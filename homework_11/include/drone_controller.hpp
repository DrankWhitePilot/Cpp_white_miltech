#pragma once

#include <array>
#include <vector>

#include "drone_link.h"
#include "types.hpp"

struct ControlDecision
{
    float accel = 0.0f;
    float turnRate = 0.0f;
    bool drop = false;
    int targetIndex = -1;
    Coord aimPoint{0.0, 0.0};
    Coord predictedTarget{0.0, 0.0};
};

class DroneController
{
public:
    void updateConfig(const dlink::DroneCfg& config);
    void updateAmmo(const dlink::AmmoCfg& ammo);
    void updateTelemetry(const dlink::Telemetry& telemetry);
    void updateTarget(const dlink::TargetPos& target);

    ControlDecision decide();

private:
    static constexpr int HISTORY_STEPS = 256;

    struct TargetTrack
    {
        bool valid = false;
        Coord pos{0.0, 0.0};
        Coord velocity{0.0, 0.0};
        double lastTime = -1.0;
        long long lastAbsoluteIndex = -1;
        int sampleCount = 0;
        std::array<Coord, HISTORY_STEPS> history{};
    };

    struct BallisticEstimate
    {
        double fallTime = 0.0;
        double horizontalDistance = 0.0;
    };

    DroneRuntime currentDroneRuntime() const;
    DroneConfig makeConfig(const DroneRuntime& drone) const;
    AmmoParams makeAmmo() const;
    BallisticEstimate estimateBallistics(
        const DroneConfig& config,
        const AmmoParams& ammo) const;

    bool buildPlanForTarget(
        int targetIndex,
        double currentTime,
        const DroneRuntime& drone,
        const DroneConfig& config,
        const BallisticEstimate& ballistic,
        AttackPlan& plan) const;

    int chooseBestPlan(
        double currentTime,
        int currentTargetIndex,
        const DroneRuntime& drone,
        const DroneConfig& config,
        const BallisticEstimate& ballistic,
        AttackPlan& bestPlan) const;

    double currentTimeSec() const;
    double targetSampleStep() const;
    double attackSpeed() const;
    double accelerationPath() const;
    double angularSpeed() const;
    double turnThreshold() const;
    double hitRadius() const;

    dlink::DroneCfg config_{};
    dlink::AmmoCfg ammo_{};
    dlink::Telemetry telemetry_{};

    bool hasConfig_ = false;
    bool hasAmmo_ = false;
    bool hasTelemetry_ = false;
    bool dropDone_ = false;

    int currentTargetIndex_ = -1;
    std::vector<TargetTrack> targets_;
};
