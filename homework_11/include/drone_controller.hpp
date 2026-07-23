#pragma once

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
    struct TargetTrack
    {
        bool valid = false;
        Coord pos{0.0, 0.0};
        Coord velocity{0.0, 0.0};
        double lastTime = -1.0;
    };

    struct BallisticEstimate
    {
        double fallTime = 0.0;
        double horizontalDistance = 0.0;
    };

    BallisticEstimate estimateBallistics() const;
    int chooseTarget(double leadTime) const;

    double currentTimeSec() const;
    double attackSpeed() const;
    double turnThreshold() const;
    double hitRadius() const;

    dlink::DroneCfg config_{};
    dlink::AmmoCfg ammo_{};
    dlink::Telemetry telemetry_{};

    bool hasConfig_ = false;
    bool hasAmmo_ = false;
    bool hasTelemetry_ = false;
    bool dropDone_ = false;

    std::vector<TargetTrack> targets_;
};
