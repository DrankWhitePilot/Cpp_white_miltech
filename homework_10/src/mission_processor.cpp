#include "mission_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

#include "model_math.hpp"

namespace
{
DroneRuntime toRuntime(const DroneTelemetry& telemetry)
{
    return {
        telemetry.pos,
        telemetry.direction,
        model::length(telemetry.speed),
        telemetry.state};
}

double estimateTargetSwitchDelay(
    const DroneConfig& config,
    const DroneRuntime& drone,
    const AttackPlan& currentPlan)
{
    const double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    if (drone.state == state_code::TURNING)
    {
        const double desiredDirection = model::directionToRadians(
            drone.position,
            currentPlan.dropPlan.firePoint,
            drone.direction);
        const double remainingTurn = std::fabs(
            model::calcTurnDeltaRadians(
                drone.direction,
                desiredDirection));
        return config.angularSpeed > model::EPS
                   ? remainingTurn / config.angularSpeed
                   : 0.0;
    }

    if (drone.state == state_code::STOPPED ||
        acceleration <= model::EPS)
    {
        return 0.0;
    }

    return drone.speed / acceleration;
}

bool isDropWindowReached(const DroneConfig& config,
                         const AttackPlan& plan,
                         const DroneRuntime& drone,
                         double& error)
{
    const Coord aimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        plan.horizontalDistance);
    error = model::length(aimPoint - plan.impactTarget);

    return drone.state == state_code::MOVING &&
           error <= config.hitRadius + model::EPS;
}
}

MissionProcessor::MissionProcessor(
    std::shared_ptr<ITargetProvider> targets,
    std::shared_ptr<DronePhysics> physics,
    std::unique_ptr<IBallisticSolver> solver,
    DroneConfig config,
    AmmoParams ammo)
    : targets_(std::move(targets)),
      physics_(std::move(physics)),
      solver_(std::move(solver)),
      config_(std::move(config)),
      ammo_(std::move(ammo)),
      state_(std::make_unique<StateStopped>()),
      activeDestination_(config_.startPos),
      activeDesiredDirection_(config_.initialDir)
{
}

bool MissionProcessor::buildPlanForTarget(
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& plan) const
{
    if (solver_ == nullptr || targetIndex < 0 ||
        targetIndex >= static_cast<int>(targetPositionHistory_.size()))
    {
        return false;
    }

    const auto& history =
        targetPositionHistory_[static_cast<std::size_t>(targetIndex)];
    if (history.empty())
    {
        return false;
    }

    BallisticResult ballistic;
    if (!solver_->solve(config_, ammo_, ballistic))
    {
        return false;
    }

    plan = model::buildAttackPlanWithBallistics(
        config_,
        history.data(),
        static_cast<int>(history.size()),
        targetIndex,
        drone,
        currentTime,
        ballistic.fallTime,
        ballistic.horizontalDistance);
    return true;
}

bool MissionProcessor::alignTargetsTo(double currentTime)
{
    if (targets_ == nullptr)
    {
        return false;
    }

    while (!stopRequested_.load())
    {
        if (!pendingTargetSnapshot_.has_value())
        {
            TargetSnapshot snapshot;
            if (targets_->tryPopSnapshot(snapshot))
            {
                pendingTargetSnapshot_ = std::move(snapshot);
            }
            else
            {
                return !currentTargets_.empty();
            }
        }

        if (pendingTargetSnapshot_->timeSecSinceStart <=
            currentTime + model::EPS)
        {
            currentTargets_ = pendingTargetSnapshot_->targets;
            pendingTargetSnapshot_.reset();
            continue;
        }

        return !currentTargets_.empty();
    }

    return false;
}

void MissionProcessor::updateTargetMotionHistory(double currentTime)
{
    const int count = static_cast<int>(currentTargets_.size());
    if (count <= 0)
    {
        return;
    }

    const std::size_t size = static_cast<std::size_t>(count);
    if (targetPositionHistory_.size() != size)
    {
        targetPositionHistory_.assign(size, {});
        targetSegmentVelocities_.assign(size, {});
        targetSampleIndices_.assign(size, -1);
    }

    const double dt = std::max(config_.arrayTimeStep, model::EPS);
    const long long sampleIndex = static_cast<long long>(
        std::floor(std::max(0.0, currentTime) / dt + model::EPS));

    for (int i = 0; i < count; ++i)
    {
        const std::size_t index = static_cast<std::size_t>(i);
        const Target& target = currentTargets_[index];
        auto& history = targetPositionHistory_[index];

        if (targetSampleIndices_[index] < 0)
        {
            const double sampleTime =
                static_cast<double>(sampleIndex) * dt;
            const double elapsed = std::clamp(
                currentTime - sampleTime,
                0.0,
                dt);
            history.push_back(target.pos - target.velocity * elapsed);
            targetSampleIndices_[index] = sampleIndex;
            targetSegmentVelocities_[index] = target.velocity;
            continue;
        }

        while (targetSampleIndices_[index] < sampleIndex)
        {
            history.push_back(
                history.back() + targetSegmentVelocities_[index] * dt);
            ++targetSampleIndices_[index];
        }

        targetSegmentVelocities_[index] = target.velocity;
    }
}

int MissionProcessor::chooseBestTarget(
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& bestPlan) const
{
    const int count = static_cast<int>(currentTargets_.size());
    if (count <= 0)
    {
        return -1;
    }

    std::vector<AttackPlan> plans(static_cast<std::size_t>(count));
    std::vector<bool> valid(static_cast<std::size_t>(count), false);
    for (int i = 0; i < count; ++i)
    {
        valid[static_cast<std::size_t>(i)] =
            buildPlanForTarget(i,
                               drone,
                               currentTime,
                               plans[static_cast<std::size_t>(i)]);
    }

    if (currentTargetIndex_ >= 0 && currentTargetIndex_ < count &&
        valid[static_cast<std::size_t>(currentTargetIndex_)] &&
        (drone.state == state_code::DECELERATING ||
         drone.state == state_code::TURNING))
    {
        bestPlan = plans[static_cast<std::size_t>(currentTargetIndex_)];
        return currentTargetIndex_;
    }

    double switchDelay = 0.0;
    if (currentTargetIndex_ >= 0 && currentTargetIndex_ < count &&
        valid[static_cast<std::size_t>(currentTargetIndex_)])
    {
        switchDelay = estimateTargetSwitchDelay(
            config_,
            drone,
            plans[static_cast<std::size_t>(currentTargetIndex_)]);
    }

    int bestFeasibleIndex = -1;
    double bestFeasibleScore = 0.0;
    int fallbackIndex = -1;
    double fallbackUncertainty = 0.0;
    double fallbackScore = 0.0;

    for (int i = 0; i < count; ++i)
    {
        if (!valid[static_cast<std::size_t>(i)])
        {
            continue;
        }

        const AttackPlan& plan = plans[static_cast<std::size_t>(i)];
        double score = plan.totalTime;
        if (currentTargetIndex_ >= 0 && i != currentTargetIndex_)
        {
            score += switchDelay;
        }

        const bool feasible =
            plan.predictionUncertainty <= config_.hitRadius + model::EPS;
        if (feasible &&
            (bestFeasibleIndex < 0 ||
             score < bestFeasibleScore - model::EPS ||
             (std::fabs(score - bestFeasibleScore) <= model::EPS &&
              i == currentTargetIndex_)))
        {
            bestFeasibleIndex = i;
            bestFeasibleScore = score;
        }

        if (fallbackIndex < 0)
        {
            fallbackIndex = i;
            fallbackUncertainty = plan.predictionUncertainty;
            fallbackScore = score;
            continue;
        }

        const bool betterFallback =
            plan.predictionUncertainty <
            fallbackUncertainty - model::EPS;
        const bool equalUncertainty =
            std::fabs(plan.predictionUncertainty -
                      fallbackUncertainty) <= model::EPS;
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
    if (bestIndex >= 0)
    {
        bestPlan = plans[static_cast<std::size_t>(bestIndex)];
    }
    return bestIndex;
}

void MissionProcessor::initializeMission(
    const DroneRuntime& drone,
    double currentTime)
{
    activePlan_ = {};
    const int bestIndex = chooseBestTarget(
        drone,
        currentTime,
        activePlan_);
    if (bestIndex < 0)
    {
        finished_ = true;
        return;
    }

    currentTargetIndex_ = bestIndex;
    mission_.targetIndex = bestIndex;
    mission_.phase = AttackPhase::PURSUIT;
    mission_.maneuverPoint = activePlan_.dropPlan.maneuverPoint;
    mission_.firePoint = activePlan_.dropPlan.firePoint;
    mission_.impactTarget = activePlan_.impactTarget;
    mission_.horizontalDistance = activePlan_.horizontalDistance;
    mission_.attackDirection = model::directionToRadians(
        activePlan_.dropPlan.needManeuver
            ? activePlan_.dropPlan.maneuverPoint
            : drone.position,
        activePlan_.dropPlan.firePoint,
        drone.direction);
    initialized_ = true;
}

void MissionProcessor::advanceStateFromTelemetry(
    const DroneTelemetry& telemetry)
{
    DroneRuntime snapshot = toRuntime(telemetry);
    if (!previousTelemetryReady_)
    {
        previousDroneSpeed_ = snapshot.speed;
        previousTelemetryReady_ = true;
    }

    DroneContext context{
        snapshot,
        config_,
        activeMotion_,
        activeDestination_,
        activeDesiredDirection_};
    context.previousSpeed = previousDroneSpeed_;
    context.completed =
        telemetry.commandCompleted &&
        telemetry.completedCommandId == activeCommandId_;
    prepareDroneStateContext(context);
    executeDroneState(state_, context);
    previousDroneSpeed_ = snapshot.speed;
}

std::uint64_t MissionProcessor::sendCommand(
    DroneMotion motion,
    Coord destination,
    double desiredDirection)
{
    if (physics_ == nullptr)
    {
        return 0;
    }

    const std::uint64_t commandId = nextCommandId_++;
    activeCommandId_ = commandId;
    activeMotion_ = motion;
    activeDestination_ = destination;
    activeDesiredDirection_ = desiredDirection;

    physics_->submitCommand({
        commandId,
        state_->code(),
        config_.angularSpeed,
        motion,
        destination,
        desiredDirection});
    return commandId;
}

void MissionProcessor::refreshCommand(
    std::uint64_t commandId,
    DroneMotion motion,
    Coord destination,
    double desiredDirection)
{
    if (physics_ == nullptr || commandId == 0)
    {
        return;
    }

    activeCommandId_ = commandId;
    activeMotion_ = motion;
    activeDestination_ = destination;
    activeDesiredDirection_ = desiredDirection;

    physics_->submitCommand({
        commandId,
        state_->code(),
        config_.angularSpeed,
        motion,
        destination,
        desiredDirection});
}

void MissionProcessor::appendStep(
    const DroneTelemetry& telemetry,
    const AttackPlan& plan)
{
    const SimStep step{
        telemetry.pos,
        telemetry.direction,
        telemetry.state,
        plan.targetIndex,
        plan.dropPlan.firePoint,
        model::calcAimPoint(telemetry.pos,
                            telemetry.direction,
                            plan.horizontalDistance),
        plan.impactTarget,
        telemetry.timeSecSinceStart};

    std::lock_guard<std::mutex> lock(stepsMutex_);
    if (steps_.size() < static_cast<std::size_t>(MAX_STEPS))
    {
        steps_.push_back(step);
    }
    else
    {
        finished_ = true;
    }
}

void MissionProcessor::processStep(const DroneTelemetry& telemetry)
{
    const double currentTime = telemetry.timeSecSinceStart;
    if (!alignTargetsTo(currentTime))
    {
        return;
    }
    updateTargetMotionHistory(currentTime);
    advanceStateFromTelemetry(telemetry);

    const DroneRuntime drone = toRuntime(telemetry);

    if (!initialized_)
    {
        initializeMission(drone, currentTime);
        if (!initialized_)
        {
            return;
        }
    }

    if (mission_.phase == AttackPhase::PURSUIT)
    {
        AttackPlan plan{};
        int bestIndex = -1;
        if (dropCandidateActive_)
        {
            bestIndex = dropCandidateTargetIndex_;
            if (!buildPlanForTarget(
                    bestIndex,
                    drone,
                    currentTime,
                    plan))
            {
                finished_ = true;
                return;
            }
        }
        else
        {
            bestIndex = chooseBestTarget(
                drone,
                currentTime,
                plan);
        }

        if (bestIndex < 0)
        {
            finished_ = true;
            return;
        }

        currentTargetIndex_ = bestIndex;
        activePlan_ = plan;
        mission_.targetIndex = bestIndex;
        mission_.firePoint = plan.dropPlan.firePoint;
        mission_.impactTarget = plan.impactTarget;
        mission_.horizontalDistance = plan.horizontalDistance;

        if (dropCandidateActive_)
        {
            double currentError = 0.0;
            const bool stillValid = isDropWindowReached(
                config_,
                plan,
                drone,
                currentError);

            if (!stillValid ||
                currentError >= dropCandidateError_ - model::EPS)
            {
                finished_ = true;
                return;
            }

            dropCandidateError_ = currentError;
            ++dropCandidateImprovements_;
            appendStep(telemetry, activePlan_);

            if (dropCandidateImprovements_ >= 2)
            {
                finished_ = true;
                return;
            }

            const double desiredDirection = model::directionToRadians(
                drone.position,
                plan.dropPlan.firePoint,
                drone.direction);
            sendCommand(DroneMotion::DYNAMIC,
                        plan.dropPlan.firePoint,
                        desiredDirection);
            return;
        }

        double currentError = 0.0;
        if (isDropWindowReached(
                config_,
                plan,
                drone,
                currentError))
        {
            dropCandidateActive_ = true;
            dropCandidateTargetIndex_ = bestIndex;
            dropCandidateError_ = currentError;
            dropCandidateImprovements_ = 0;
            appendStep(telemetry, activePlan_);

            const double desiredDirection = model::directionToRadians(
                drone.position,
                plan.dropPlan.firePoint,
                drone.direction);
            sendCommand(DroneMotion::DYNAMIC,
                        plan.dropPlan.firePoint,
                        desiredDirection);
            return;
        }

        if (plan.dropPlan.needManeuver)
        {
            mission_.phase = AttackPhase::TO_MANEUVER;
            mission_.maneuverPoint = plan.dropPlan.maneuverPoint;
            mission_.attackDirection = model::directionToRadians(
                mission_.maneuverPoint,
                mission_.firePoint,
                drone.direction);
            phaseCommandId_ = sendCommand(
                DroneMotion::STOP_AT_POINT,
                mission_.maneuverPoint,
                mission_.attackDirection);
            appendStep(telemetry, activePlan_);
            return;
        }

        const double desiredDirection = model::directionToRadians(
            drone.position,
            plan.dropPlan.firePoint,
            drone.direction);
        sendCommand(DroneMotion::DYNAMIC,
                    plan.dropPlan.firePoint,
                    desiredDirection);
        appendStep(telemetry, activePlan_);
        return;
    }

    if (mission_.phase == AttackPhase::TO_MANEUVER)
    {
        if (telemetry.commandCompleted &&
            telemetry.completedCommandId == phaseCommandId_)
        {
            mission_.phase = AttackPhase::ALIGN_ATTACK;
            phaseCommandId_ = sendCommand(
                DroneMotion::TURN_IN_PLACE,
                telemetry.pos,
                mission_.attackDirection);
        }
        else
        {
            refreshCommand(
                phaseCommandId_,
                DroneMotion::STOP_AT_POINT,
                mission_.maneuverPoint,
                mission_.attackDirection);
        }
        appendStep(telemetry, activePlan_);
        return;
    }

    if (mission_.phase == AttackPhase::ALIGN_ATTACK)
    {
        if (telemetry.commandCompleted &&
            telemetry.completedCommandId == phaseCommandId_)
        {
            mission_.phase = AttackPhase::ATTACK_RUN;
            phaseCommandId_ = sendCommand(
                DroneMotion::LOCKED_ATTACK_RUN,
                mission_.firePoint,
                mission_.attackDirection);
        }
        else
        {
            refreshCommand(
                phaseCommandId_,
                DroneMotion::TURN_IN_PLACE,
                telemetry.pos,
                mission_.attackDirection);
        }
        appendStep(telemetry, activePlan_);
        return;
    }

    if (telemetry.commandCompleted &&
        telemetry.completedCommandId == phaseCommandId_)
    {
        finished_ = true;
    }
    else
    {
        refreshCommand(
            phaseCommandId_,
            DroneMotion::LOCKED_ATTACK_RUN,
            mission_.firePoint,
            mission_.attackDirection);
    }
    appendStep(telemetry, activePlan_);
}

void MissionProcessor::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const double dt = std::max(config_.simTimeStep, 0.001);
    const double scale = std::max(config_.timeScale, 0.001);
    const auto period = std::chrono::duration<double>(dt / scale);
    auto nextTick = std::chrono::steady_clock::now();

    while (!stopRequested_.load() && !finished_)
    {
        processStep(physics_->getTelemetry());
        nextTick += std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(nextTick);
    }
}

bool MissionProcessor::isThreadReady() const
{
    return ready_.load();
}

void MissionProcessor::start()
{
    started_.store(true);
}

void MissionProcessor::stop()
{
    stopRequested_.store(true);
}

const SimStep* MissionProcessor::getSteps() const
{
    return steps_.data();
}

int MissionProcessor::getStepCount() const
{
    std::lock_guard<std::mutex> lock(stepsMutex_);
    return static_cast<int>(steps_.size());
}
