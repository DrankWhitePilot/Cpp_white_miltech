#include "mission_processor.hpp"

#include <vector>

#include <algorithm>
#include <cmath>

#include "drone_state.hpp"
#include "interfaces.hpp"
#include "model_math.hpp"

namespace
{
double estimateTargetSwitchDelay(
    const DroneConfig& config,
    const DroneRuntime& drone,
    const AttackPlan& currentPlan)
{
    double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    if (drone.state == state_code::TURNING)
    {
        double desiredDirection = model::directionToRadians(
            drone.position,
            currentPlan.dropPlan.firePoint,
            drone.direction);
        double remainingTurn = std::fabs(model::calcTurnDeltaRadians(
            drone.direction,
            desiredDirection));
        if (config.angularSpeed > model::EPS)
        {
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

SimStep makeStep(
    const DroneRuntime& drone,
    int targetIndex,
    Coord dropPoint,
    double horizontalDistance,
    Coord predictedTarget)
{
    return {
        drone.position,
        drone.direction,
        drone.state,
        targetIndex,
        dropPoint,
        model::calcAimPoint(
            drone.position,
            drone.direction,
            horizontalDistance),
        predictedTarget};
}

SimStep advanceOneDynamicStep(
    const DroneConfig& config,
    const AttackPlan& plan,
    DroneRuntime& drone)
{
    const double dt = config.simTimeStep;
    const double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    double desiredDirection = model::directionToRadians(
        drone.position,
        plan.dropPlan.firePoint,
        drone.direction);
    double turnDelta = model::calcTurnDeltaRadians(
        drone.direction,
        desiredDirection);

    if (std::fabs(turnDelta) > config.turnThreshold + model::EPS)
    {
        if (drone.speed > model::EPS && acceleration > model::EPS)
        {
            const double oldSpeed = drone.speed;
            double newSpeed = oldSpeed - acceleration * dt;
            if (newSpeed < 0.0)
            {
                newSpeed = 0.0;
            }

            const double moveDistance =
                0.5 * (oldSpeed + newSpeed) * dt;
            drone.position = drone.position +
                             model::directionVector(drone.direction) *
                                 moveDistance;
            drone.speed = newSpeed;
            drone.state = state_code::DECELERATING;

            return makeStep(
                drone,
                plan.targetIndex,
                plan.dropPlan.firePoint,
                plan.horizontalDistance,
                plan.impactTarget);
        }

        drone.speed = 0.0;
        const double maxTurn = config.angularSpeed * dt;
        if (std::fabs(turnDelta) <= maxTurn + model::EPS)
        {
            drone.direction = desiredDirection;
        }
        else
        {
            drone.direction = model::normalizeAngleTwoPi(
                drone.direction +
                (turnDelta > 0.0 ? maxTurn : -maxTurn));
        }
        drone.state = state_code::TURNING;

        return makeStep(
            drone,
            plan.targetIndex,
            plan.dropPlan.firePoint,
            plan.horizontalDistance,
            plan.impactTarget);
    }

    Coord oldVelocity =
        model::directionVector(drone.direction) * drone.speed;
    Coord desiredVelocity =
        model::directionVector(desiredDirection) * config.attackSpeed;
    Coord deltaVelocity = desiredVelocity - oldVelocity;
    const double deltaVelocityLength = model::length(deltaVelocity);
    const double maxDeltaVelocity =
        acceleration > model::EPS ? acceleration * dt : 0.0;

    Coord newVelocity = oldVelocity;
    if (deltaVelocityLength > model::EPS &&
        maxDeltaVelocity > model::EPS)
    {
        if (deltaVelocityLength <= maxDeltaVelocity + model::EPS)
        {
            newVelocity = desiredVelocity;
        }
        else
        {
            newVelocity = oldVelocity +
                          deltaVelocity *
                              (maxDeltaVelocity / deltaVelocityLength);
        }
    }

    const double newSpeed = model::length(newVelocity);
    Coord averageVelocity = (oldVelocity + newVelocity) * 0.5;
    drone.position = drone.position + averageVelocity * dt;

    if (newSpeed > model::EPS)
    {
        drone.direction = model::normalizeAngleTwoPi(
            std::atan2(newVelocity.y, newVelocity.x));
    }

    if (newSpeed > drone.speed + model::EPS)
    {
        drone.state = state_code::ACCELERATING;
    }
    else if (newSpeed + model::EPS < drone.speed)
    {
        drone.state = state_code::DECELERATING;
    }
    else
    {
        drone.state = state_code::MOVING;
    }
    drone.speed = newSpeed;

    return makeStep(
        drone,
        plan.targetIndex,
        plan.dropPlan.firePoint,
        plan.horizontalDistance,
        plan.impactTarget);
}

SimStep advanceTowardStopPoint(
    const DroneConfig& config,
    int targetIndex,
    Coord destination,
    Coord displayedDropPoint,
    double horizontalDistance,
    Coord predictedTarget,
    DroneRuntime& drone)
{
    const double dt = config.simTimeStep;
    const double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);
    Coord toDestination = destination - drone.position;
    double remaining = model::length(toDestination);

    if (remaining <= model::EPS && drone.speed <= model::EPS)
    {
        drone.position = destination;
        drone.speed = 0.0;
        drone.state = state_code::STOPPED;
        return makeStep(
            drone,
            targetIndex,
            displayedDropPoint,
            horizontalDistance,
            predictedTarget);
    }

    double desiredDirection = model::directionToRadians(
        drone.position,
        destination,
        drone.direction);
    double turnDelta = model::calcTurnDeltaRadians(
        drone.direction,
        desiredDirection);

    if (std::fabs(turnDelta) > model::EPS)
    {
        if (drone.speed > model::EPS && acceleration > model::EPS)
        {
            double oldSpeed = drone.speed;
            double newSpeed = std::max(
                0.0,
                oldSpeed - acceleration * dt);
            double distance = 0.5 * (oldSpeed + newSpeed) * dt;
            drone.position = drone.position +
                             model::directionVector(drone.direction) *
                                 distance;
            drone.speed = newSpeed;
            drone.state = newSpeed > model::EPS
                              ? state_code::DECELERATING
                              : state_code::STOPPED;
            return makeStep(
                drone,
                targetIndex,
                displayedDropPoint,
                horizontalDistance,
                predictedTarget);
        }

        drone.speed = 0.0;
        double maxTurn = config.angularSpeed * dt;
        if (std::fabs(turnDelta) <= maxTurn + model::EPS)
        {
            drone.direction = desiredDirection;
        }
        else
        {
            drone.direction = model::normalizeAngleTwoPi(
                drone.direction +
                (turnDelta > 0.0 ? maxTurn : -maxTurn));
        }
        drone.state = state_code::TURNING;
        return makeStep(
            drone,
            targetIndex,
            displayedDropPoint,
            horizontalDistance,
            predictedTarget);
    }

    if (acceleration > model::EPS &&
        drone.speed <= acceleration * dt + model::EPS)
    {
        double stoppingDistance =
            drone.speed * drone.speed / (2.0 * acceleration);
        if (std::fabs(stoppingDistance - remaining) <= 1e-7)
        {
            drone.position = destination;
            drone.speed = 0.0;
            drone.state = state_code::STOPPED;
            return makeStep(
                drone,
                targetIndex,
                displayedDropPoint,
                horizontalDistance,
                predictedTarget);
        }
    }

    double oldSpeed = drone.speed;
    double minimumNextSpeed = std::max(
        0.0,
        oldSpeed - acceleration * dt);
    double maximumNextSpeed = std::min(
        config.attackSpeed,
        oldSpeed + acceleration * dt);

    double discriminant =
        acceleration * acceleration * dt * dt -
        4.0 * (acceleration * dt * oldSpeed -
               2.0 * acceleration * remaining);
    double stopLimitedSpeed = 0.0;
    if (discriminant > 0.0)
    {
        stopLimitedSpeed =
            (-acceleration * dt + std::sqrt(discriminant)) / 2.0;
    }
    stopLimitedSpeed = std::max(0.0, stopLimitedSpeed);

    double newSpeed = std::min(maximumNextSpeed, stopLimitedSpeed);
    if (newSpeed < minimumNextSpeed)
    {
        newSpeed = minimumNextSpeed;
    }

    double moveDistance = 0.5 * (oldSpeed + newSpeed) * dt;
    if (moveDistance > remaining &&
        moveDistance - remaining <= 1e-7)
    {
        moveDistance = remaining;
    }

    drone.position = drone.position +
                     model::directionVector(desiredDirection) *
                         moveDistance;
    drone.direction = desiredDirection;
    drone.speed = newSpeed;

    if (newSpeed > oldSpeed + model::EPS)
    {
        drone.state = state_code::ACCELERATING;
    }
    else if (newSpeed + model::EPS < oldSpeed)
    {
        drone.state = state_code::DECELERATING;
    }
    else if (newSpeed <= model::EPS)
    {
        drone.state = state_code::STOPPED;
    }
    else
    {
        drone.state = state_code::MOVING;
    }

    return makeStep(
        drone,
        targetIndex,
        displayedDropPoint,
        horizontalDistance,
        predictedTarget);
}

SimStep advanceTurnInPlace(
    const DroneConfig& config,
    const MissionRuntime& mission,
    DroneRuntime& drone,
    bool& aligned)
{
    aligned = false;
    drone.speed = 0.0;
    double delta = model::calcTurnDeltaRadians(
        drone.direction,
        mission.attackDirection);
    double maxTurn = config.angularSpeed * config.simTimeStep;
    if (std::fabs(delta) <= maxTurn + model::EPS)
    {
        drone.direction = mission.attackDirection;
        aligned = true;
    }
    else
    {
        drone.direction = model::normalizeAngleTwoPi(
            drone.direction +
            (delta > 0.0 ? maxTurn : -maxTurn));
    }
    drone.state = state_code::TURNING;
    return makeStep(
        drone,
        mission.targetIndex,
        mission.firePoint,
        mission.horizontalDistance,
        mission.impactTarget);
}

SimStep advanceLockedAttackRun(
    const DroneConfig& config,
    const MissionRuntime& mission,
    DroneRuntime& drone,
    bool& reachedFirePoint)
{
    reachedFirePoint = false;
    double acceleration = model::calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);
    double dt = config.simTimeStep;
    Coord attackVector = model::directionVector(
        mission.attackDirection);
    double remaining = model::length(
        mission.firePoint - drone.position);
    double oldSpeed = drone.speed;
    double newSpeed = std::min(
        config.attackSpeed,
        oldSpeed + acceleration * dt);
    double moveDistance = 0.5 * (oldSpeed + newSpeed) * dt;

    if (moveDistance + model::EPS >= remaining)
    {
        drone.position = mission.firePoint;
        double reachableSpeed = std::sqrt(std::max(
            0.0,
            oldSpeed * oldSpeed +
                2.0 * acceleration * remaining));
        drone.speed = std::min(
            config.attackSpeed,
            reachableSpeed);
        drone.direction = mission.attackDirection;
        drone.state = drone.speed >= config.attackSpeed - model::EPS
                          ? state_code::MOVING
                          : state_code::ACCELERATING;
        reachedFirePoint = true;
    }
    else
    {
        drone.position =
            drone.position + attackVector * moveDistance;
        drone.speed = newSpeed;
        drone.direction = mission.attackDirection;
        drone.state = newSpeed >= config.attackSpeed - model::EPS
                          ? state_code::MOVING
                          : state_code::ACCELERATING;
    }

    return makeStep(
        drone,
        mission.targetIndex,
        mission.firePoint,
        mission.horizontalDistance,
        mission.impactTarget);
}

bool isDropWindowReached(
    const DroneConfig& config,
    const AttackPlan& plan,
    const DroneRuntime& drone,
    Coord& currentAimPoint)
{
    currentAimPoint = model::calcAimPoint(
        drone.position,
        drone.direction,
        plan.horizontalDistance);
    if (drone.state != state_code::MOVING ||
        drone.speed < config.attackSpeed - model::EPS)
    {
        return false;
    }
    return model::length(currentAimPoint - plan.impactTarget) <=
           config.hitRadius + model::EPS;
}
}

MissionProcessor::MissionProcessor(
    std::unique_ptr<ITargetProvider> targets,
    std::unique_ptr<IBallisticSolver> solver,
    std::unique_ptr<IConfigLoader> loader)
    : targets_(std::move(targets)),
      solver_(std::move(solver)),
      loader_(std::move(loader)),
      state_(std::make_unique<StateStopped>())
{
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::buildPlanForTarget(
    int targetIndex,
    double currentTime,
    AttackPlan& plan) const
{
    if (targets_ == nullptr ||
        solver_ == nullptr ||
        loader_ == nullptr ||
        targetIndex < 0 ||
        targetIndex >= targets_->getTargetCount())
    {
        return false;
    }

    return solver_->solve(
        loader_->getConfig(),
        loader_->getAmmoParams(),
        targets_->getTarget(targetIndex),
        targets_->getTimeSteps(),
        targetIndex,
        drone_,
        currentTime,
        plan);
}

int MissionProcessor::chooseBestTargetFromState(
    double currentTime,
    int currentTargetIndex,
    AttackPlan& bestPlan) const
{
    if (targets_ == nullptr || loader_ == nullptr)
    {
        return -1;
    }

    int targetCount = targets_->getTargetCount();
    if (targetCount <= 0)
    {
        return -1;
    }

    std::vector<AttackPlan> plans(targetCount);
    std::vector<bool> validPlans(targetCount, false);
    for (int i = 0; i < targetCount; ++i)
    {
        validPlans[i] = buildPlanForTarget(i, currentTime, plans[i]);
    }

    if (currentTargetIndex >= 0 &&
        currentTargetIndex < targetCount &&
        validPlans[currentTargetIndex] &&
        (drone_.state == state_code::DECELERATING ||
         drone_.state == state_code::TURNING))
    {
        bestPlan = plans[currentTargetIndex];
        return currentTargetIndex;
    }

    const DroneConfig& config = loader_->getConfig();
    double switchDelay = 0.0;
    if (currentTargetIndex >= 0 &&
        currentTargetIndex < targetCount &&
        validPlans[currentTargetIndex])
    {
        switchDelay = estimateTargetSwitchDelay(
            config,
            drone_,
            plans[currentTargetIndex]);
    }

    int bestFeasibleIndex = -1;
    double bestFeasibleScore = 0.0;
    int fallbackIndex = -1;
    double fallbackUncertainty = 0.0;
    double fallbackScore = 0.0;

    for (int i = 0; i < targetCount; ++i)
    {
        if (!validPlans[i])
        {
            continue;
        }

        double score = plans[i].totalTime;
        if (currentTargetIndex >= 0 && i != currentTargetIndex)
        {
            score += switchDelay;
        }

        bool feasible =
            plans[i].predictionUncertainty <=
            config.hitRadius + model::EPS;
        if (feasible &&
            (bestFeasibleIndex < 0 ||
             score < bestFeasibleScore - model::EPS ||
             (std::fabs(score - bestFeasibleScore) <= model::EPS &&
              i == currentTargetIndex)))
        {
            bestFeasibleIndex = i;
            bestFeasibleScore = score;
        }

        if (fallbackIndex < 0)
        {
            fallbackIndex = i;
            fallbackUncertainty = plans[i].predictionUncertainty;
            fallbackScore = score;
            continue;
        }

        bool betterFallback =
            plans[i].predictionUncertainty <
            fallbackUncertainty - model::EPS;
        bool equalUncertainty =
            std::fabs(plans[i].predictionUncertainty -
                      fallbackUncertainty) <= model::EPS;
        if (betterFallback ||
            (equalUncertainty &&
             (score < fallbackScore - model::EPS ||
              (std::fabs(score - fallbackScore) <= model::EPS &&
               i == currentTargetIndex))))
        {
            fallbackIndex = i;
            fallbackUncertainty = plans[i].predictionUncertainty;
            fallbackScore = score;
        }
    }

    int bestIndex = bestFeasibleIndex >= 0
                        ? bestFeasibleIndex
                        : fallbackIndex;
    if (bestIndex >= 0)
    {
        bestPlan = plans[bestIndex];
    }

    return bestIndex;
}

void MissionProcessor::synchronizeState(int requestedState)
{
    if (state_ == nullptr || loader_ == nullptr)
    {
        return;
    }

    DroneContext context{
        drone_,
        loader_->getConfig(),
        requestedState};
    auto next = state_->execute(context);
    if (next)
    {
        state_ = std::move(next);
    }
    drone_.state = state_->code();
}

bool MissionProcessor::appendStep(const SimStep& step)
{
    if (stepCount_ >= MAX_STEPS)
    {
        finished_ = true;
        return false;
    }
    synchronizeState(step.state);
    steps_.push_back(step);
    ++stepCount_;
    return true;
}

void MissionProcessor::initializeRuntime()
{
    initialized_ = false;
    finished_ = true;
    stepCount_ = 0;
    steps_.clear();
    currentTargetIndex_ = -1;
    mission_ = {};
    state_ = std::make_unique<StateStopped>();

    if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr)
    {
        return;
    }

    const DroneConfig& config = loader_->getConfig();
    drone_ = {
        config.startPos,
        config.initialDir,
        0.0,
        state_code::STOPPED};

    AttackPlan initialPlan = {};
    int initialBestIndex = chooseBestTargetFromState(
        0.0,
        -1,
        initialPlan);
    if (initialBestIndex < 0)
    {
        return;
    }

    Coord initialWaypoint = initialPlan.dropPlan.needManeuver
                                ? initialPlan.dropPlan.maneuverPoint
                                : initialPlan.dropPlan.firePoint;
    double initialDesiredDirection = model::directionToRadians(
        drone_.position,
        initialWaypoint,
        drone_.direction);
    if (std::fabs(model::calcTurnDeltaRadians(
            drone_.direction,
            initialDesiredDirection)) <=
        config.turnThreshold + model::EPS)
    {
        drone_.direction = initialDesiredDirection;
    }

    mission_ = {
        initialPlan.targetIndex,
        AttackPhase::PURSUIT,
        initialPlan.dropPlan.maneuverPoint,
        initialPlan.dropPlan.firePoint,
        initialPlan.predictedTarget,
        initialDesiredDirection,
        initialPlan.horizontalDistance};

    currentTargetIndex_ = initialPlan.targetIndex;
    finished_ = false;
    initialized_ = true;

    appendStep({
        drone_.position,
        drone_.direction,
        drone_.state,
        initialPlan.targetIndex,
        initialPlan.dropPlan.firePoint,
        model::calcAimPoint(
            drone_.position,
            drone_.direction,
            initialPlan.horizontalDistance),
        initialPlan.dropPlan.needManeuver
            ? initialPlan.predictedTarget
            : initialPlan.impactTarget});
}

int MissionProcessor::init()
{
    if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr)
    {
        return 1;
    }
    if (targets_->load() != 0)
    {
        return 1;
    }
    if (loader_->load() != 0)
    {
        return 1;
    }

    initializeRuntime();
    return initialized_ ? 0 : 1;
}

bool MissionProcessor::hasNext() const
{
    return initialized_ && !finished_ && stepCount_ < MAX_STEPS;
}

void MissionProcessor::step()
{
    if (!hasNext())
    {
        return;
    }

    const DroneConfig& config = loader_->getConfig();
    double currentTime =
        (stepCount_ - 1) * config.simTimeStep;

    if (mission_.phase == AttackPhase::ATTACK_RUN)
    {
        bool reachedFirePoint = false;
        SimStep next = advanceLockedAttackRun(
            config,
            mission_,
            drone_,
            reachedFirePoint);
        appendStep(next);
        if (reachedFirePoint)
        {
            finished_ = true;
        }
        return;
    }

    if (mission_.phase == AttackPhase::ALIGN_ATTACK)
    {
        bool aligned = false;
        SimStep next = advanceTurnInPlace(
            config,
            mission_,
            drone_,
            aligned);
        appendStep(next);
        if (aligned)
        {
            mission_.phase = AttackPhase::ATTACK_RUN;
        }
        return;
    }

    if (mission_.phase == AttackPhase::TO_MANEUVER)
    {
        SimStep next = advanceTowardStopPoint(
            config,
            mission_.targetIndex,
            mission_.maneuverPoint,
            mission_.firePoint,
            mission_.horizontalDistance,
            mission_.impactTarget,
            drone_);
        appendStep(next);

        if (drone_.speed <= model::EPS &&
            model::length(
                drone_.position - mission_.maneuverPoint) <= model::EPS)
        {
            mission_.phase = AttackPhase::ALIGN_ATTACK;
        }
        return;
    }

    AttackPlan plan = {};
    int bestIndex = chooseBestTargetFromState(
        currentTime,
        currentTargetIndex_,
        plan);
    if (bestIndex < 0)
    {
        finished_ = true;
        return;
    }
    currentTargetIndex_ = bestIndex;

    if (plan.dropPlan.needManeuver)
    {
        mission_.targetIndex = plan.targetIndex;
        mission_.phase = AttackPhase::TO_MANEUVER;
        mission_.maneuverPoint = plan.dropPlan.maneuverPoint;
        mission_.firePoint = plan.dropPlan.firePoint;
        mission_.impactTarget = plan.predictedTarget;
        mission_.attackDirection = model::directionToRadians(
            mission_.maneuverPoint,
            mission_.firePoint,
            drone_.direction);
        mission_.horizontalDistance = plan.horizontalDistance;

        appendStep(advanceTowardStopPoint(
            config,
            mission_.targetIndex,
            mission_.maneuverPoint,
            mission_.firePoint,
            mission_.horizontalDistance,
            mission_.impactTarget,
            drone_));
        return;
    }

    Coord currentAimPoint = {};
    if (isDropWindowReached(
            config,
            plan,
            drone_,
            currentAimPoint))
    {
        double currentError = model::length(
            currentAimPoint - plan.impactTarget);
        DroneRuntime nextDrone = drone_;
        SimStep nextStep = advanceOneDynamicStep(
            config,
            plan,
            nextDrone);

        Coord* targetPath = targets_->getTarget(plan.targetIndex);
        int timeSteps = targets_->getTimeSteps();
        model::ObservedTargetState observedNow =
            model::observeTargetFromPast(
                targetPath,
                timeSteps,
                currentTime,
                config.arrayTimeStep);
        Coord predictionResidual = model::estimatePredictionResidual(
            targetPath,
            timeSteps,
            currentTime,
            config.arrayTimeStep,
            plan.fallTime);
        Coord nextImpactTarget = model::predictObservedTarget(
                                     observedNow,
                                     plan.fallTime + config.simTimeStep) +
                                 predictionResidual;
        Coord nextAimPoint = model::calcAimPoint(
            nextDrone.position,
            nextDrone.direction,
            plan.horizontalDistance);
        double nextError = model::length(
            nextAimPoint - nextImpactTarget);
        bool velocityIsObserved =
            currentTime + model::EPS >= config.arrayTimeStep;
        bool nextStepIsBetter =
            velocityIsObserved &&
            nextDrone.state == state_code::MOVING &&
            nextDrone.speed >= config.attackSpeed - model::EPS &&
            nextError < currentError - model::EPS &&
            nextError <= config.hitRadius + model::EPS;

        if (nextStepIsBetter && stepCount_ < MAX_STEPS)
        {
            double nextTimeToDrop = 0.0;
            DropPlan nextDropPlan = model::calculateDynamicDropPlan(
                config,
                nextDrone,
                nextImpactTarget,
                plan.horizontalDistance,
                nextTimeToDrop);
            (void)nextTimeToDrop;

            bool useSecondStep = false;
            DroneRuntime secondDrone = nextDrone;
            SimStep secondStep = nextStep;
            Coord secondImpactTarget = nextImpactTarget;
            Coord secondAimPoint = nextAimPoint;
            DropPlan secondDropPlan = nextDropPlan;

            if (stepCount_ + 1 < MAX_STEPS)
            {
                AttackPlan secondPlan = plan;
                secondPlan.dropPlan = nextDropPlan;
                secondPlan.impactTarget = nextImpactTarget;

                SimStep candidateSecondStep = advanceOneDynamicStep(
                    config,
                    secondPlan,
                    secondDrone);
                Coord candidateSecondImpactTarget =
                    model::predictObservedTarget(
                        observedNow,
                        plan.fallTime +
                            2.0 * config.simTimeStep) +
                    predictionResidual;
                Coord candidateSecondAimPoint = model::calcAimPoint(
                    secondDrone.position,
                    secondDrone.direction,
                    plan.horizontalDistance);
                double secondError = model::length(
                    candidateSecondAimPoint -
                    candidateSecondImpactTarget);

                double secondTimeToDrop = 0.0;
                DropPlan candidateSecondDropPlan =
                    model::calculateDynamicDropPlan(
                        config,
                        secondDrone,
                        candidateSecondImpactTarget,
                        plan.horizontalDistance,
                        secondTimeToDrop);
                (void)secondTimeToDrop;

                if (!candidateSecondDropPlan.needManeuver &&
                    secondDrone.state == state_code::MOVING &&
                    secondDrone.speed >=
                        config.attackSpeed - model::EPS &&
                    secondError < nextError - model::EPS &&
                    secondError <= config.hitRadius + model::EPS)
                {
                    useSecondStep = true;
                    secondStep = candidateSecondStep;
                    secondImpactTarget = candidateSecondImpactTarget;
                    secondAimPoint = candidateSecondAimPoint;
                    secondDropPlan = candidateSecondDropPlan;
                }
            }

            nextStep.targetIndex = plan.targetIndex;
            nextStep.dropPoint = nextDropPlan.firePoint;
            nextStep.aimPoint = nextAimPoint;
            nextStep.predictedTarget = nextImpactTarget;
            drone_ = nextDrone;
            appendStep(nextStep);

            if (useSecondStep)
            {
                secondStep.targetIndex = plan.targetIndex;
                secondStep.dropPoint = secondDropPlan.firePoint;
                secondStep.aimPoint = secondAimPoint;
                secondStep.predictedTarget = secondImpactTarget;
                drone_ = secondDrone;
                appendStep(secondStep);
            }
        }
        else if (stepCount_ > 0)
        {
            steps_[stepCount_ - 1].targetIndex = plan.targetIndex;
            steps_[stepCount_ - 1].dropPoint =
                plan.dropPlan.firePoint;
            steps_[stepCount_ - 1].aimPoint = currentAimPoint;
            steps_[stepCount_ - 1].predictedTarget =
                plan.impactTarget;
        }

        finished_ = true;
        return;
    }

    appendStep(advanceOneDynamicStep(config, plan, drone_));
}

void MissionProcessor::reset()
{    initializeRuntime();
}

void MissionProcessor::changeSolver(
    std::unique_ptr<IBallisticSolver> solver)
{
    if (solver != nullptr)
    {
        solver_ = std::move(solver);
    }
}

const SimStep* MissionProcessor::getSteps() const
{
    return steps_.data();
}

int MissionProcessor::getStepCount() const
{
    return stepCount_;
}
