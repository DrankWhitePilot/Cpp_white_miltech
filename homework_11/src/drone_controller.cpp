#include "drone_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "drone_state.hpp"
#include "model_math.hpp"

namespace
{
constexpr double DEFAULT_ATTACK_SPEED = 10.0;
constexpr double DEFAULT_ACCELERATION_PATH = 10.0;
constexpr double DEFAULT_ANGULAR_SPEED = 1.0;
constexpr double DEFAULT_TURN_THRESHOLD = 0.10;
constexpr double DEFAULT_HIT_RADIUS = 3.0;
constexpr double DEFAULT_TIME_STEP = 0.1;

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

float clampFloat(double value)
{
    return static_cast<float>(clampDouble(value, -1.0, 1.0));
}

Coord fromTelemetryPos(const dlink::Telemetry& telemetry)
{
    return {telemetry.x, telemetry.y};
}

Coord fromTargetPos(const dlink::TargetPos& target)
{
    return {target.x, target.y};
}

int positiveModulo(long long value, int modulo)
{
    int result = static_cast<int>(value % modulo);
    if (result < 0) {
        result += modulo;
    }
    return result;
}

std::string fixedString(const char* data, std::size_t size)
{
    std::size_t len = 0;
    while (len < size && data[len] != '\0') {
        ++len;
    }
    return std::string(data, len);
}

double estimateTargetSwitchDelay(
    const DroneConfig& config,
    const DroneRuntime& drone,
    const AttackPlan& currentPlan)
{
    const double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    if (drone.state == state_code::TURNING) {
        const double desiredDirection = model::directionToRadians(
            drone.position,
            currentPlan.dropPlan.firePoint,
            drone.direction);
        const double remainingTurn = std::fabs(model::calcTurnDeltaRadians(
            drone.direction,
            desiredDirection));
        if (config.angularSpeed > model::EPS) {
            return remainingTurn / config.angularSpeed;
        }
        return 0.0;
    }

    if (drone.state == state_code::STOPPED || acceleration <= model::EPS) {
        return 0.0;
    }

    return drone.speed / acceleration;
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
        targets_.resize(static_cast<std::size_t>(target.id) + 1U);
    }

    TargetTrack& track = targets_[target.id];

    const double now = currentTimeSec();
    const double sampleStep = targetSampleStep();
    const long long absoluteIndex = static_cast<long long>(
        std::floor(now / sampleStep + model::EPS));
    const int index = positiveModulo(absoluteIndex, HISTORY_STEPS);
    const Coord pos = fromTargetPos(target);

    if (!track.valid) {
        track.history.fill(pos);
        track.pos = pos;
        track.velocity = {0.0, 0.0};
        track.lastTime = now;
        track.lastAbsoluteIndex = absoluteIndex;
        track.sampleCount = 1;
        track.valid = true;
        return;
    }

    if (now > track.lastTime + model::EPS) {
        track.velocity = (pos - track.pos) / (now - track.lastTime);
    }

    if (absoluteIndex > track.lastAbsoluteIndex) {
        const long long totalDelta = absoluteIndex - track.lastAbsoluteIndex;
        const long long cappedDelta =
            std::min<long long>(totalDelta, HISTORY_STEPS);

        const Coord previousPos = track.pos;
        for (long long k = 1; k <= cappedDelta; ++k) {
            const double alpha = static_cast<double>(k) /
                                 static_cast<double>(totalDelta);
            const Coord interpolated =
                previousPos * (1.0 - alpha) + pos * alpha;
            const int writeIndex = positiveModulo(
                track.lastAbsoluteIndex + k,
                HISTORY_STEPS);
            track.history[writeIndex] = interpolated;
        }

        track.sampleCount = std::min(
            HISTORY_STEPS,
            track.sampleCount + static_cast<int>(cappedDelta));
    } else {
        track.history[index] = pos;
    }

    track.pos = pos;
    track.lastTime = now;
    track.lastAbsoluteIndex = absoluteIndex;
    track.valid = true;
}

double DroneController::currentTimeSec() const
{
    return hasTelemetry_ ? static_cast<double>(telemetry_.t_ms) / 1000.0 : 0.0;
}

double DroneController::targetSampleStep() const
{
    if (hasConfig_ && config_.timeStep > model::EPS) {
        return config_.timeStep;
    }
    return DEFAULT_TIME_STEP;
}

double DroneController::attackSpeed() const
{
    if (hasConfig_ && config_.attackSpeed > model::EPS) {
        return config_.attackSpeed;
    }
    return DEFAULT_ATTACK_SPEED;
}

double DroneController::accelerationPath() const
{
    if (hasConfig_ && config_.accelerationPath > model::EPS) {
        return config_.accelerationPath;
    }
    return DEFAULT_ACCELERATION_PATH;
}

double DroneController::angularSpeed() const
{
    if (hasConfig_ && config_.angularSpeed > model::EPS) {
        return config_.angularSpeed;
    }
    return DEFAULT_ANGULAR_SPEED;
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

DroneRuntime DroneController::currentDroneRuntime() const
{
    return {
        fromTelemetryPos(telemetry_),
        telemetry_.dir,
        telemetry_.speed,
        telemetry_.state};
}

DroneConfig DroneController::makeConfig(const DroneRuntime& drone) const
{
    DroneConfig config{};
    config.startPos = drone.position;
    config.altitude = telemetry_.z;
    config.initialDir = drone.direction;
    config.attackSpeed = attackSpeed();
    config.accelPath = accelerationPath();
    config.ammoName = fixedString(ammo_.name, sizeof(ammo_.name));
    config.arrayTimeStep = targetSampleStep();
    config.simTimeStep = targetSampleStep();
    config.physicsTimeStep = targetSampleStep();
    config.timeScale =
        hasConfig_ && config_.timeScale > model::EPS ? config_.timeScale : 1.0;
    config.hitRadius = hitRadius();
    config.angularSpeed = angularSpeed();
    config.turnThreshold = turnThreshold();
    return config;
}

AmmoParams DroneController::makeAmmo() const
{
    AmmoParams ammo{};
    ammo.name = fixedString(ammo_.name, sizeof(ammo_.name));
    ammo.mass = hasAmmo_ && ammo_.mass > model::EPS ? ammo_.mass : 1.0;
    ammo.drag = hasAmmo_ && ammo_.drag >= 0.0f ? ammo_.drag : 0.0;
    ammo.lift = hasAmmo_ && ammo_.lift >= 0.0f ? ammo_.lift : 0.0;
    return ammo;
}

DroneController::BallisticEstimate DroneController::estimateBallistics(
    const DroneConfig& config,
    const AmmoParams& ammo) const
{
    BallisticEstimate estimate{};

    if (!hasAmmo_ || config.altitude <= model::EPS) {
        return estimate;
    }

    estimate.fallTime = model::solveFallTime(
        ammo,
        config.altitude,
        config.attackSpeed);
    if (estimate.fallTime <= model::EPS) {
        return {};
    }

    estimate.horizontalDistance = model::calcHorizontalDistance(
        ammo,
        estimate.fallTime,
        config.attackSpeed);
    if (estimate.horizontalDistance <= model::EPS) {
        return {};
    }

    return estimate;
}

bool DroneController::buildPlanForTarget(
    int targetIndex,
    double currentTime,
    const DroneRuntime& drone,
    const DroneConfig& config,
    const BallisticEstimate& ballistic,
    AttackPlan& plan) const
{
    if (targetIndex < 0 ||
        targetIndex >= static_cast<int>(targets_.size()) ||
        !targets_[static_cast<std::size_t>(targetIndex)].valid ||
        targets_[static_cast<std::size_t>(targetIndex)].sampleCount <= 0 ||
        ballistic.fallTime <= model::EPS ||
        ballistic.horizontalDistance <= model::EPS) {
        return false;
    }

    const TargetTrack& target = targets_[static_cast<std::size_t>(targetIndex)];
    plan = model::buildAttackPlanWithBallistics(
        config,
        target.history.data(),
        HISTORY_STEPS,
        targetIndex,
        drone,
        currentTime,
        ballistic.fallTime,
        ballistic.horizontalDistance);

    return plan.fallTime > model::EPS &&
           plan.horizontalDistance > model::EPS &&
           plan.targetIndex == targetIndex;
}

int DroneController::chooseBestPlan(
    double currentTime,
    int currentTargetIndex,
    const DroneRuntime& drone,
    const DroneConfig& config,
    const BallisticEstimate& ballistic,
    AttackPlan& bestPlan) const
{
    const int targetCount = static_cast<int>(targets_.size());
    if (targetCount <= 0) {
        return -1;
    }

    std::vector<AttackPlan> plans(static_cast<std::size_t>(targetCount));
    std::vector<bool> validPlans(static_cast<std::size_t>(targetCount), false);

    for (int i = 0; i < targetCount; ++i) {
        validPlans[static_cast<std::size_t>(i)] = buildPlanForTarget(
            i,
            currentTime,
            drone,
            config,
            ballistic,
            plans[static_cast<std::size_t>(i)]);
    }

    if (currentTargetIndex >= 0 &&
        currentTargetIndex < targetCount &&
        validPlans[static_cast<std::size_t>(currentTargetIndex)] &&
        (drone.state == state_code::DECELERATING ||
         drone.state == state_code::TURNING)) {
        bestPlan = plans[static_cast<std::size_t>(currentTargetIndex)];
        return currentTargetIndex;
    }

    double switchDelay = 0.0;
    if (currentTargetIndex >= 0 &&
        currentTargetIndex < targetCount &&
        validPlans[static_cast<std::size_t>(currentTargetIndex)]) {
        switchDelay = estimateTargetSwitchDelay(
            config,
            drone,
            plans[static_cast<std::size_t>(currentTargetIndex)]);
    }

    int bestFeasibleIndex = -1;
    double bestFeasibleScore = 0.0;
    int fallbackIndex = -1;
    double fallbackUncertainty = 0.0;
    double fallbackScore = 0.0;

    for (int i = 0; i < targetCount; ++i) {
        if (!validPlans[static_cast<std::size_t>(i)]) {
            continue;
        }

        double score = plans[static_cast<std::size_t>(i)].totalTime;
        if (currentTargetIndex >= 0 && i != currentTargetIndex) {
            score += switchDelay;
        }

        const bool feasible =
            plans[static_cast<std::size_t>(i)].predictionUncertainty <=
            config.hitRadius + model::EPS;
        if (feasible &&
            (bestFeasibleIndex < 0 ||
             score < bestFeasibleScore - model::EPS ||
             (std::fabs(score - bestFeasibleScore) <= model::EPS &&
              i == currentTargetIndex))) {
            bestFeasibleIndex = i;
            bestFeasibleScore = score;
        }

        if (fallbackIndex < 0) {
            fallbackIndex = i;
            fallbackUncertainty =
                plans[static_cast<std::size_t>(i)].predictionUncertainty;
            fallbackScore = score;
            continue;
        }

        const bool betterFallback =
            plans[static_cast<std::size_t>(i)].predictionUncertainty <
            fallbackUncertainty - model::EPS;
        const bool equalUncertainty =
            std::fabs(
                plans[static_cast<std::size_t>(i)].predictionUncertainty -
                fallbackUncertainty) <= model::EPS;

        if (betterFallback ||
            (equalUncertainty &&
             (score < fallbackScore - model::EPS ||
              (std::fabs(score - fallbackScore) <= model::EPS &&
               i == currentTargetIndex)))) {
            fallbackIndex = i;
            fallbackUncertainty =
                plans[static_cast<std::size_t>(i)].predictionUncertainty;
            fallbackScore = score;
        }
    }

    const int bestIndex = bestFeasibleIndex >= 0
                              ? bestFeasibleIndex
                              : fallbackIndex;
    if (bestIndex >= 0) {
        bestPlan = plans[static_cast<std::size_t>(bestIndex)];
    }

    return bestIndex;
}

ControlDecision DroneController::decide()
{
    ControlDecision decision{};

    if (!hasTelemetry_) {
        return decision;
    }

    const DroneRuntime drone = currentDroneRuntime();
    const DroneConfig config = makeConfig(drone);
    const AmmoParams ammo = makeAmmo();
    const BallisticEstimate ballistic = estimateBallistics(config, ammo);

    if (ballistic.fallTime <= model::EPS ||
        ballistic.horizontalDistance <= model::EPS) {
        decision.accel = 0.3f;
        decision.turnRate = 0.0f;
        return decision;
    }

    AttackPlan plan{};
    const int targetIndex = chooseBestPlan(
        currentTimeSec(),
        currentTargetIndex_,
        drone,
        config,
        ballistic,
        plan);

    decision.targetIndex = targetIndex;
    if (targetIndex < 0) {
        decision.accel = 0.5f;
        decision.turnRate = 0.0f;
        return decision;
    }

    currentTargetIndex_ = targetIndex;
    decision.predictedTarget = plan.impactTarget;

    const bool goToManeuver =
        plan.dropPlan.needManeuver &&
        model::length(plan.dropPlan.maneuverPoint - drone.position) >
            config.hitRadius * 2.0;

    const Coord controlPoint = goToManeuver
                                   ? plan.dropPlan.maneuverPoint
                                   : plan.dropPlan.firePoint;

    const double desiredDirection = model::directionToRadians(
        drone.position,
        controlPoint,
        drone.direction);
    const double angleError = model::calcTurnDeltaRadians(
        drone.direction,
        desiredDirection);
    const double absAngleError = std::fabs(angleError);

    decision.turnRate = clampFloat(
        angleError / std::max(config.turnThreshold, 0.05));

    if (absAngleError > 0.85) {
        decision.accel =
            drone.speed > config.attackSpeed * 0.35 ? -0.25f : 0.10f;
    } else if (drone.speed < config.attackSpeed * 0.97) {
        decision.accel = 1.0f;
    } else if (drone.speed > config.attackSpeed * 1.05) {
        decision.accel = -0.25f;
    } else {
        decision.accel = 0.0f;
    }

    const Coord currentAimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        plan.horizontalDistance);
    decision.aimPoint = currentAimPoint;

    const bool dropWindowReached =
        drone.state == state_code::MOVING &&
        drone.speed >= config.attackSpeed - model::EPS &&
        model::length(currentAimPoint - plan.impactTarget) <=
            config.hitRadius + model::EPS;

    if (!dropDone_ && dropWindowReached) {
        decision.drop = true;
        dropDone_ = true;
    }

    return decision;
}
