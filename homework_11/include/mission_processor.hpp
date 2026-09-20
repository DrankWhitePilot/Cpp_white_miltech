#pragma once

#include <vector>

#include "drone_link.h"
#include "mission_guidance.hpp"
#include "types.hpp"

class MissionProcessor
{
public:
    void updateConfig(const dlink::DroneCfg& config);
    void updateAmmo(const dlink::AmmoCfg& ammo);
    void updateTelemetry(const dlink::Telemetry& telemetry);
    void updateTarget(const dlink::TargetPos& target);

    MissionDecision decide();

private:
    struct TargetTrack
    {
        bool valid = false;
        std::vector<Coord> path;
    };

    bool ready() const;
    double currentTimeSec() const;

    DroneConfig makeConfig() const;
    AmmoParams makeAmmo() const;
    DroneRuntime makeRuntime() const;

    bool solveBallistics(const DroneConfig& config,
                         const AmmoParams& ammo,
                         BallisticResult& result) const;

    bool buildPlanForTarget(int targetIndex,
                            const DroneConfig& config,
                            const DroneRuntime& drone,
                            const BallisticResult& ballistic,
                            AttackPlan& plan) const;

    int chooseTarget(const DroneConfig& config,
                     const DroneRuntime& drone,
                     const BallisticResult& ballistic,
                     AttackPlan& bestPlan) const;

    bool currentDropWindow(int targetIndex,
                           const DroneConfig& config,
                           const AmmoParams& ammo,
                           const DroneRuntime& drone,
                           MissionDecision& decision) const;

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
