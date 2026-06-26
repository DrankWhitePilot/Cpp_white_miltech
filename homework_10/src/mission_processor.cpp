#include "mission_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

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

bool isDropWindowReached(const DroneConfig& config,
                         const AttackPlan& plan,
                         const DroneRuntime& drone)
{
    const Coord aimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        plan.horizontalDistance);

    return drone.state == state_code::MOVING &&
           drone.speed >= config.attackSpeed - model::EPS &&
           model::length(aimPoint - plan.impactTarget) <=
               config.hitRadius + model::EPS;
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
      state_(std::make_unique<StateStopped>())
{
}

bool MissionProcessor::buildPlanForTarget(
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& plan) const
{
    if (targets_ == nullptr || solver_ == nullptr ||
        targetIndex < 0 || targetIndex >= targets_->getTargetCount())
    {
        return false;
    }

    const Target target = targets_->getTarget(targetIndex);
    return solver_->solve(config_,
                          ammo_,
                          target,
                          targetIndex,
                          drone,
                          currentTime,
                          plan);
}

int MissionProcessor::chooseBestTarget(
    const DroneRuntime& drone,
    double currentTime,
    AttackPlan& bestPlan) const
{
    if (targets_ == nullptr)
    {
        return -1;
    }

    const int count = targets_->getTargetCount();
    int bestIndex = -1;
    double bestScore = 0.0;

    for (int i = 0; i < count; ++i)
    {
        AttackPlan plan{};
        if (!buildPlanForTarget(i, drone, currentTime, plan))
        {
            continue;
        }

        const double score = plan.totalTime;
        if (bestIndex < 0 || score < bestScore - model::EPS ||
            (std::fabs(score - bestScore) <= model::EPS &&
             i == currentTargetIndex_))
        {
            bestIndex = i;
            bestScore = score;
            bestPlan = plan;
        }
    }
    return bestIndex;
}

void MissionProcessor::initializeMission(const DroneRuntime& drone)
{
    activePlan_ = {};
    const int bestIndex = chooseBestTarget(drone, 0.0, activePlan_);
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

void MissionProcessor::sendCommand(
    DroneMotion motion,
    Coord destination,
    double desiredDirection,
    int state)
{
    if (physics_ == nullptr)
    {
        return;
    }

    physics_->submitCommand({
        state,
        config_.angularSpeed,
        motion,
        destination,
        desiredDirection});
}

void MissionProcessor::appendStep(
    const DroneTelemetry& telemetry,
    const AttackPlan& plan)
{
    SimStep step{
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

void MissionProcessor::syncStateObject(int stateCode)
{
    if (state_ != nullptr && state_->code() == stateCode)
    {
        return;
    }

    switch (stateCode)
    {
    case state_code::ACCELERATING:
        state_ = std::make_unique<StateAccelerating>();
        break;
    case state_code::DECELERATING:
        state_ = std::make_unique<StateDecelerating>();
        break;
    case state_code::TURNING:
        state_ = std::make_unique<StateTurning>();
        break;
    case state_code::MOVING:
        state_ = std::make_unique<StateMoving>();
        break;
    default:
        state_ = std::make_unique<StateStopped>();
        break;
    }
}

void MissionProcessor::processStep(const DroneTelemetry& telemetry)
{
    const DroneRuntime drone = toRuntime(telemetry);
    syncStateObject(drone.state);

    if (!initialized_)
    {
        initializeMission(drone);
        if (!initialized_)
        {
            return;
        }
    }

    const double currentTime = telemetry.timeSecSinceStart;

    if (mission_.phase == AttackPhase::PURSUIT)
    {
        AttackPlan plan{};
        const int bestIndex = chooseBestTarget(
            drone,
            currentTime,
            plan);
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

        if (plan.dropPlan.needManeuver)
        {
            mission_.phase = AttackPhase::TO_MANEUVER;
            mission_.maneuverPoint = plan.dropPlan.maneuverPoint;
            mission_.attackDirection = model::directionToRadians(
                mission_.maneuverPoint,
                mission_.firePoint,
                drone.direction);
            sendCommand(DroneMotion::STOP_AT_POINT,
                        mission_.maneuverPoint,
                        mission_.attackDirection,
                        drone.state);
        }
        else if (isDropWindowReached(config_, plan, drone))
        {
            finished_ = true;
        }
        else
        {
            const double desiredDirection = model::directionToRadians(
                drone.position,
                plan.dropPlan.firePoint,
                drone.direction);
            sendCommand(DroneMotion::DYNAMIC,
                        plan.dropPlan.firePoint,
                        desiredDirection,
                        drone.state);
        }

        appendStep(telemetry, activePlan_);
        return;
    }

    if (mission_.phase == AttackPhase::TO_MANEUVER)
    {
        if (telemetry.commandCompleted)
        {
            mission_.phase = AttackPhase::ALIGN_ATTACK;
        }
        sendCommand(DroneMotion::STOP_AT_POINT,
                    mission_.maneuverPoint,
                    mission_.attackDirection,
                    drone.state);
        appendStep(telemetry, activePlan_);
        return;
    }

    if (mission_.phase == AttackPhase::ALIGN_ATTACK)
    {
        if (telemetry.commandCompleted)
        {
            mission_.phase = AttackPhase::ATTACK_RUN;
        }
        sendCommand(DroneMotion::TURN_IN_PLACE,
                    telemetry.pos,
                    mission_.attackDirection,
                    drone.state);
        appendStep(telemetry, activePlan_);
        return;
    }

    if (telemetry.commandCompleted)
    {
        finished_ = true;
    }
    else
    {
        sendCommand(DroneMotion::LOCKED_ATTACK_RUN,
                    mission_.firePoint,
                    mission_.attackDirection,
                    drone.state);
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
    const auto sleepDuration = std::chrono::duration<double>(dt / scale);

    while (!stopRequested_.load() && !finished_)
    {
        processStep(physics_->getTelemetry());
        std::this_thread::sleep_for(sleepDuration);
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
