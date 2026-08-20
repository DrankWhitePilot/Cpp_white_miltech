#include "drone_controller.hpp"

#include <algorithm>
#include <cmath>

#include "model_math.hpp"

namespace
{
constexpr double DEFAULT_SIM_TIME_STEP = 0.1;

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

float clampFloat(double value)
{
    return static_cast<float>(clampDouble(value, -1.0, 1.0));
}
}

void DroneController::updateConfig(const dlink::DroneCfg& config)
{
    config_ = config;
    hasConfig_ = true;
}

void DroneController::updateTelemetry(const dlink::Telemetry& telemetry)
{
    telemetry_ = telemetry;
    hasTelemetry_ = true;
}

ControlCommand DroneController::control(
    const MissionDecision& mission) const
{
    ControlCommand command{};

    if (!hasConfig_ || !hasTelemetry_ || !mission.valid) {
        return command;
    }

    const Coord dronePosition{telemetry_.x, telemetry_.y};
    const double desiredDirection = model::directionToRadians(
        dronePosition,
        mission.destination,
        telemetry_.dir);
    const double delta = model::calcTurnDeltaRadians(
        telemetry_.dir,
        desiredDirection);

    const double dt = config_.timeStep > model::EPS
                          ? config_.timeStep
                          : DEFAULT_SIM_TIME_STEP;
    const double maxTurnThisStep = std::max(
        model::EPS,
        static_cast<double>(config_.angularSpeed) * dt);
    const bool turnRequired =
        std::fabs(delta) >
        static_cast<double>(config_.turnThreshold) + model::EPS;

    if (turnRequired) {
        if (telemetry_.speed > model::EPS) {
            command.accel = -1.0f;
            command.turnRate = 0.0f;
        } else {
            command.accel = 0.0f;
            command.turnRate = clampFloat(delta / maxTurnThisStep);
        }
    } else {
        command.turnRate = clampFloat(delta / maxTurnThisStep);

        if (telemetry_.speed < mission.desiredSpeed - model::EPS) {
            command.accel = 1.0f;
        } else if (telemetry_.speed > mission.desiredSpeed + model::EPS) {
            command.accel = -1.0f;
        } else {
            command.accel = 0.0f;
        }
    }

    return command;
}
