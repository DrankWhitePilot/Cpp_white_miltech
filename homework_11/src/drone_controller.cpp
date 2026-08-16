#include "drone_controller.hpp"

#include <algorithm>
#include <cmath>

#include "model_math.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double G = 9.80665;
constexpr double DEFAULT_ATTACK_SPEED = 25.0;
constexpr double DEFAULT_TURN_THRESHOLD = 0.10;
constexpr double DEFAULT_HIT_RADIUS = 3.0;

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

float clampFloat(double value)
{
    return static_cast<float>(clampDouble(value, -1.0, 1.0));
}

double normalizeAngle(double angle)
{
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

Coord fromTelemetryPos(const dlink::Telemetry& telemetry)
{
    return {telemetry.x, telemetry.y};
}

Coord fromTargetPos(const dlink::TargetPos& target)
{
    return {target.x, target.y};
}
}

void DroneController::updateConfig(const dlink::DroneCfg& config)
{
    config_ = config;
    hasConfig_ = true;
}

void DroneController::updateAmmo(const dlink::AmmoCfg& ammo)
{
    ammo_ = ammo;
    hasAmmo_ = true;

    if (ammo.nTargets > 0 && targets_.size() < ammo.nTargets) {
        targets_.resize(ammo.nTargets);
    }
}

void DroneController::updateTelemetry(const dlink::Telemetry& telemetry)
{
    telemetry_ = telemetry;
    hasTelemetry_ = true;
}

void DroneController::updateTarget(const dlink::TargetPos& target)
{
    if (target.id >= targets_.size()) {
        targets_.resize(static_cast<size_t>(target.id) + 1U);
    }

    TargetTrack& track = targets_[target.id];

    const double now = currentTimeSec();
    const Coord pos = fromTargetPos(target);

    if (track.valid && track.lastTime >= 0.0 && now > track.lastTime) {
        const double dt = now - track.lastTime;
        track.velocity = (pos - track.pos) / dt;
    }

    track.pos = pos;
    track.lastTime = now;
    track.valid = true;
}

double DroneController::currentTimeSec() const
{
    return hasTelemetry_ ? static_cast<double>(telemetry_.t_ms) / 1000.0 : 0.0;
}

double DroneController::attackSpeed() const
{
    if (hasConfig_ && config_.attackSpeed > model::EPS) {
        return config_.attackSpeed;
    }
    return DEFAULT_ATTACK_SPEED;
}

double DroneController::turnThreshold() const
{
    if (hasConfig_ && config_.turnThreshold > model::EPS) {
        return config_.turnThreshold;
    }
    return DEFAULT_TURN_THRESHOLD;
}

double DroneController::hitRadius() const
{
    if (hasAmmo_ && ammo_.hitRadius > model::EPS) {
        return ammo_.hitRadius;
    }
    return DEFAULT_HIT_RADIUS;
}

DroneController::BallisticEstimate DroneController::estimateBallistics() const
{
    BallisticEstimate estimate{};

    if (!hasTelemetry_ || telemetry_.z <= 0.0f) {
        return estimate;
    }

    const double mass = (hasAmmo_ && ammo_.mass > 0.001f) ? ammo_.mass : 1.0;
    const double drag = (hasAmmo_ && ammo_.drag > 0.0f) ? ammo_.drag : 0.0;
    const double lift = (hasAmmo_ && ammo_.lift > 0.0f) ? ammo_.lift : 0.0;

    double z = telemetry_.z;
    double verticalSpeed = 0.0;
    double horizontalSpeed = std::max(0.0, static_cast<double>(telemetry_.speed));
    double horizontalDistance = 0.0;

    constexpr double dt = 0.002;
    constexpr double maxTime = 30.0;

    for (double t = 0.0; t < maxTime; t += dt) {
        const double verticalDrag = drag * verticalSpeed * std::fabs(verticalSpeed) / mass;
        double verticalAccel = G - verticalDrag - lift / mass;
        if (verticalAccel < G * 0.25) {
            verticalAccel = G * 0.25;
        }

        verticalSpeed += verticalAccel * dt;
        z -= verticalSpeed * dt;

        const double horizontalDrag = drag * horizontalSpeed * std::fabs(horizontalSpeed) / mass;
        horizontalSpeed -= horizontalDrag * dt;
        if (horizontalSpeed < 0.0) {
            horizontalSpeed = 0.0;
        }

        horizontalDistance += horizontalSpeed * dt;

        if (z <= 0.0) {
            estimate.fallTime = t + dt;
            estimate.horizontalDistance = horizontalDistance;
            return estimate;
        }
    }

    estimate.fallTime = std::sqrt((2.0 * telemetry_.z) / G);
    estimate.horizontalDistance = std::max(0.0, static_cast<double>(telemetry_.speed)) *
                                  estimate.fallTime;
    return estimate;
}

int DroneController::chooseTarget(double leadTime) const
{
    if (!hasTelemetry_) {
        return -1;
    }

    const Coord dronePos = fromTelemetryPos(telemetry_);

    int bestIndex = -1;
    double bestScore = 0.0;

    for (size_t i = 0; i < targets_.size(); ++i) {
        if (!targets_[i].valid) {
            continue;
        }

        const Coord predicted = targets_[i].pos + targets_[i].velocity * leadTime;
        const double score = model::length(predicted - dronePos);

        if (bestIndex < 0 || score < bestScore) {
            bestIndex = static_cast<int>(i);
            bestScore = score;
        }
    }

    return bestIndex;
}

ControlDecision DroneController::decide()
{
    ControlDecision decision{};

    if (!hasTelemetry_) {
        return decision;
    }

    const BallisticEstimate ballistic = estimateBallistics();
    if (ballistic.fallTime <= model::EPS) {
        decision.accel = 0.3f;
        decision.turnRate = 0.0f;
        return decision;
    }

    const Coord dronePos = fromTelemetryPos(telemetry_);
    const double speedForApproach = std::max(
        attackSpeed(),
        static_cast<double>(telemetry_.speed));

    const double leadTime = ballistic.fallTime + 100.0 / std::max(1.0, speedForApproach);

    const int targetIndex = chooseTarget(leadTime);
    decision.targetIndex = targetIndex;

    if (targetIndex < 0) {
        decision.accel = 0.5f;
        decision.turnRate = 0.0f;
        return decision;
    }

    const TargetTrack& target = targets_[static_cast<size_t>(targetIndex)];

    const double distanceToTargetNow = model::length(target.pos - dronePos);
    const double timeToArrive = distanceToTargetNow / std::max(1.0, speedForApproach);
    const Coord predictedForSteering =
        target.pos + target.velocity * (timeToArrive + ballistic.fallTime);

    decision.predictedTarget = predictedForSteering;

    const double desiredDirection = std::atan2(
        predictedForSteering.y - dronePos.y,
        predictedForSteering.x - dronePos.x);

    const double angleError = normalizeAngle(desiredDirection - telemetry_.dir);
    const double absAngleError = std::fabs(angleError);

    decision.turnRate = clampFloat(angleError / std::max(turnThreshold(), 0.05));

    if (absAngleError > 0.85) {
        decision.accel = telemetry_.speed > attackSpeed() * 0.35 ? -0.25f : 0.10f;
    } else if (telemetry_.speed < attackSpeed() * 0.97) {
        decision.accel = 1.0f;
    } else if (telemetry_.speed > attackSpeed() * 1.05) {
        decision.accel = -0.25f;
    } else {
        decision.accel = 0.0f;
    }

    const Coord predictedAtImpact = target.pos + target.velocity * ballistic.fallTime;
    const Coord aimPoint = model::calcAimPoint(
        dronePos,
        telemetry_.dir,
        ballistic.horizontalDistance);

    decision.aimPoint = aimPoint;

    const double dropMiss = model::length(aimPoint - predictedAtImpact);
    const bool stableAttack =
        telemetry_.speed >= attackSpeed() * 0.80 &&
        absAngleError <= 0.25;

    if (!dropDone_ && stableAttack && dropMiss <= hitRadius()) {
        decision.drop = true;
        dropDone_ = true;
    }

    return decision;
}
