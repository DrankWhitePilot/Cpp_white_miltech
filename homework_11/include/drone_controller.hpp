#pragma once

#include "drone_link.h"
#include "mission_guidance.hpp"

struct ControlCommand
{
    float accel = 0.0f;
    float turnRate = 0.0f;
};

class DroneController
{
public:
    void updateConfig(const dlink::DroneCfg& config);
    void updateTelemetry(const dlink::Telemetry& telemetry);

    ControlCommand control(const MissionDecision& mission) const;

private:
    dlink::DroneCfg config_{};
    dlink::Telemetry telemetry_{};
    bool hasConfig_ = false;
    bool hasTelemetry_ = false;
};
