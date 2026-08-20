#include "mission_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "drone_state.hpp"
#include "model_math.hpp"
#include "table_solver.hpp"

namespace
{
constexpr double DEFAULT_SIM_TIME_STEP = 0.1;

#ifndef HW11_DATA_DIR
#define HW11_DATA_DIR "data"
#endif

Coord telemetryPos(const dlink::Telemetry& telemetry)
{
    return {telemetry.x, telemetry.y};
}

Coord targetPos(const dlink::TargetPos& target)
{
    return {target.x, target.y};
}

std::string fixedName(const char* data, std::size_t size)
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

    if (drone.state == state_code::STOPPED ||
        acceleration <= model::EPS)
    {
        return 0.0;
    }

    return drone.speed / acceleration;
}
}

void MissionProcessor::updateConfig(const dlink::DroneCfg& config)
{
    config_ = config;
    hasConfig_ = true;
}

void MissionProcessor::updateAmmo(const dlink::AmmoCfg& ammo)
{
    ammo_ = ammo;
    hasAmmo_ = true;

    if (ammo.nTargets > 0 && targets_.size() < ammo.nTargets) {
        targets_.resize(ammo.nTargets);
    }
}

void MissionProcessor::updateTelemetry(const dlink::Telemetry& telemetry)
{
    telemetry_ = telemetry;
    hasTelemetry_ = true;
}

void MissionProcessor::updateTarget(const dlink::TargetPos& target)
{
    if (target.id >= targets_.size()) {
        targets_.resize(static_cast<std::size_t>(target.id) + 1U);
    }

    TargetTrack& track = targets_[target.id];
    track.path.push_back(targetPos(target));
    track.valid = track.path.size() >= 2U;
}

bool MissionProcessor::ready() const
{
    return hasConfig_ && hasAmmo_ && hasTelemetry_;
}

double MissionProcessor::currentTimeSec() const
{
    return hasTelemetry_
               ? static_cast<double>(telemetry_.t_ms) / 1000.0
               : 0.0;
}

DroneConfig MissionProcessor::makeConfig() const
{
    DroneConfig config{};
    config.startPos = telemetryPos(telemetry_);
    config.altitude = telemetry_.z;
    config.initialDir = telemetry_.dir;
    config.attackSpeed = config_.attackSpeed;
    config.accelPath = config_.accelerationPath;
    config.ammoName = fixedName(ammo_.name, sizeof(ammo_.name));
    config.arrayTimeStep = config_.timeStep > model::EPS
                               ? config_.timeStep
                               : DEFAULT_SIM_TIME_STEP;
    config.simTimeStep = config.arrayTimeStep;
    config.physicsTimeStep = config.arrayTimeStep;
    config.timeScale = config_.timeScale;
    config.hitRadius = ammo_.hitRadius;
    config.angularSpeed = config_.angularSpeed;
    config.turnThreshold = config_.turnThreshold;
    return config;
}

AmmoParams MissionProcessor::makeAmmo() const
{
    AmmoParams ammo{};
    ammo.name = fixedName(ammo_.name, sizeof(ammo_.name));
    ammo.mass = ammo_.mass;
    ammo.drag = ammo_.drag;
    ammo.lift = ammo_.lift;
    return ammo;
}

DroneRuntime MissionProcessor::makeRuntime() const
{
    return {
        telemetryPos(telemetry_),
        telemetry_.dir,
        telemetry_.speed,
        static_cast<int>(telemetry_.state)};
}

bool MissionProcessor::solveBallistics(
    const DroneConfig& config,
    const AmmoParams& ammo,
    BallisticResult& result) const
{
    static TableSolver tableSolver(HW11_DATA_DIR "/ballistic_table.txt");
    static TableSolver localTableSolver("data/ballistic_table.txt");

    if (tableSolver.isLoaded() && tableSolver.solve(config, ammo, result)) {
        return true;
    }

    if (localTableSolver.isLoaded() &&
        localTableSolver.solve(config, ammo, result))
    {
        return true;
    }

    result.fallTime = model::solveFallTime(
        ammo,
        config.altitude,
        config.attackSpeed);

    if (result.fallTime <= model::EPS) {
        return false;
    }

    result.horizontalDistance = model::calcHorizontalDistance(
        ammo,
        result.fallTime,
        config.attackSpeed);

    return std::isfinite(result.horizontalDistance) &&
           result.horizontalDistance >= 0.0;
}

bool MissionProcessor::buildPlanForTarget(
    int targetIndex,
    const DroneConfig& config,
    const DroneRuntime& drone,
    const BallisticResult& ballistic,
    AttackPlan& plan) const
{
    if (targetIndex < 0 ||
        targetIndex >= static_cast<int>(targets_.size())) {
        return false;
    }

    const TargetTrack& track =
        targets_[static_cast<std::size_t>(targetIndex)];

    if (!track.valid || track.path.size() < 2U) {
        return false;
    }

    const double pathCurrentTime =
        static_cast<double>(track.path.size() - 1U) * config.arrayTimeStep;
    const double currentTime = std::min(currentTimeSec(), pathCurrentTime);

    plan = model::buildAttackPlanWithBallistics(
        config,
        track.path.data(),
        static_cast<int>(track.path.size()),
        targetIndex,
        drone,
        currentTime,
        ballistic.fallTime,
        ballistic.horizontalDistance);

    return plan.fallTime > model::EPS &&
           std::isfinite(plan.horizontalDistance) &&
           plan.horizontalDistance >= 0.0;
}

int MissionProcessor::chooseTarget(
    const DroneConfig& config,
    const DroneRuntime& drone,
    const BallisticResult& ballistic,
    AttackPlan& bestPlan) const
{
    const int targetCount = static_cast<int>(targets_.size());
    if (targetCount <= 0) {
        return -1;
    }

    std::vector<AttackPlan> plans(static_cast<std::size_t>(targetCount));
    std::vector<bool> validPlans(static_cast<std::size_t>(targetCount), false);

    for (int i = 0; i < targetCount; ++i) {
        validPlans[static_cast<std::size_t>(i)] =
            buildPlanForTarget(
                i,
                config,
                drone,
                ballistic,
                plans[static_cast<std::size_t>(i)]);
    }

    if (currentTargetIndex_ >= 0 &&
        currentTargetIndex_ < targetCount &&
        validPlans[static_cast<std::size_t>(currentTargetIndex_)] &&
        (drone.state == state_code::DECELERATING ||
         drone.state == state_code::TURNING))
    {
        bestPlan = plans[static_cast<std::size_t>(currentTargetIndex_)];
        return currentTargetIndex_;
    }

    double switchDelay = 0.0;
    if (currentTargetIndex_ >= 0 &&
        currentTargetIndex_ < targetCount &&
        validPlans[static_cast<std::size_t>(currentTargetIndex_)])
    {
        switchDelay = estimateTargetSwitchDelay(
            config,
            drone,
            plans[static_cast<std::size_t>(currentTargetIndex_)]);
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

        const AttackPlan& plan = plans[static_cast<std::size_t>(i)];

        double score = plan.totalTime;
        if (currentTargetIndex_ >= 0 && i != currentTargetIndex_) {
            score += switchDelay;
        }

        const bool feasible =
            plan.predictionUncertainty <= config.hitRadius + model::EPS;

        if (feasible &&
            (bestFeasibleIndex < 0 ||
             score < bestFeasibleScore - model::EPS ||
             (std::fabs(score - bestFeasibleScore) <= model::EPS &&
              i == currentTargetIndex_)))
        {
            bestFeasibleIndex = i;
            bestFeasibleScore = score;
        }

        if (fallbackIndex < 0) {
            fallbackIndex = i;
            fallbackUncertainty = plan.predictionUncertainty;
            fallbackScore = score;
            continue;
        }

        const bool betterFallback =
            plan.predictionUncertainty < fallbackUncertainty - model::EPS;
        const bool equalUncertainty =
            std::fabs(plan.predictionUncertainty - fallbackUncertainty) <=
            model::EPS;

        if (betterFallback ||
            (equalUncertainty &&
             (score < fallbackScore - model::EPS ||
              (std::fabs(score - fallbackScore) <= model::EPS &&
               i == currentTargetIndex_))))
        {
            fallbackIndex = i;
            fallbackUncertainty = plan.predictionUncertainty;
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

bool MissionProcessor::currentDropWindow(
    int targetIndex,
    const DroneConfig& config,
    const AmmoParams& ammo,
    const DroneRuntime& drone,
    MissionDecision& decision) const
{
    if (drone.speed <= model::EPS ||
        targetIndex < 0 ||
        targetIndex >= static_cast<int>(targets_.size()))
    {
        return false;
    }

    const TargetTrack& track =
        targets_[static_cast<std::size_t>(targetIndex)];
    if (!track.valid || track.path.size() < 2U) {
        return false;
    }

    DroneConfig releaseConfig = config;
    releaseConfig.attackSpeed = drone.speed;

    BallisticResult releaseBallistic{};
    if (!solveBallistics(releaseConfig, ammo, releaseBallistic)) {
        return false;
    }

    const double pathCurrentTime =
        static_cast<double>(track.path.size() - 1U) * config.arrayTimeStep;
    const double currentTime = std::min(currentTimeSec(), pathCurrentTime);
    const model::ObservedTargetState observed = model::observeTargetFromPast(
        track.path.data(),
        static_cast<int>(track.path.size()),
        currentTime,
        config.arrayTimeStep);
    const Coord predictionResidual = model::estimatePredictionResidual(
        track.path.data(),
        static_cast<int>(track.path.size()),
        currentTime,
        config.arrayTimeStep,
        releaseBallistic.fallTime);

    decision.predictedTarget = model::predictObservedTarget(
                                   observed,
                                   releaseBallistic.fallTime) +
                               predictionResidual;
    decision.aimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        releaseBallistic.horizontalDistance);

    const double dropError =
        model::length(decision.aimPoint - decision.predictedTarget);
    return dropError <= config.hitRadius + model::EPS;
}

MissionDecision MissionProcessor::decide()
{
    MissionDecision decision{};

    if (!ready()) {
        return decision;
    }

    const DroneConfig config = makeConfig();
    const AmmoParams ammo = makeAmmo();
    const DroneRuntime drone = makeRuntime();

    BallisticResult ballistic{};
    if (!solveBallistics(config, ammo, ballistic)) {
        return decision;
    }

    AttackPlan plan{};
    const int targetIndex = chooseTarget(config, drone, ballistic, plan);
    if (targetIndex < 0) {
        return decision;
    }

    currentTargetIndex_ = targetIndex;

    decision.valid = true;
    decision.destination = plan.dropPlan.needManeuver
                               ? plan.dropPlan.maneuverPoint
                               : plan.dropPlan.firePoint;
    decision.desiredSpeed = config.attackSpeed;
    decision.targetIndex = plan.targetIndex;
    decision.predictedTarget = plan.impactTarget;
    decision.aimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        plan.horizontalDistance);

    const bool dropWindow = currentDropWindow(
        targetIndex,
        config,
        ammo,
        drone,
        decision);

    if (!dropDone_ && dropWindow) {
        decision.drop = true;
        dropDone_ = true;
    }

    return decision;
}
