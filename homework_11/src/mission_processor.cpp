#include "mission_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

#include "drone_state.hpp"
#include "model_math.hpp"
#include "thread_safe_target_provider.hpp"

namespace
{
DroneRuntime runtimeFromTelemetry(const DroneTelemetry& telemetry)
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
    const DroneTelemetry& telemetry,
    int targetIndex,
    Coord dropPoint,
    double horizontalDistance,
    Coord predictedTarget)
{
    return {
        telemetry.pos,
        telemetry.direction,
        telemetry.state,
        targetIndex,
        dropPoint,
        model::calcAimPoint(
            telemetry.pos,
            telemetry.direction,
            horizontalDistance),
        predictedTarget,
        telemetry.timeSecSinceStart};
}

SimStep makeStep(
    const DroneRuntime& drone,
    double timeSecSinceStart,
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
        predictedTarget,
        timeSecSinceStart};
}

SimStep simulateOneDynamicStep(
    const DroneConfig& config,
    const AttackPlan& plan,
    DroneRuntime& drone,
    std::unique_ptr<IDroneState>& state,
    double timeSecSinceStart)
{
    DroneContext context{
        drone,
        config,
        DroneMotion::DYNAMIC,
        plan.dropPlan.firePoint,
        0.0};
    executeDroneState(state, context);

    return makeStep(
        drone,
        timeSecSinceStart + config.simTimeStep,
        plan.targetIndex,
        plan.dropPlan.firePoint,
        plan.horizontalDistance,
        plan.impactTarget);
}

DronePhysicsResult executePhysicsCommand(
    DronePhysics& physics,
    std::unique_ptr<IDroneState>& state,
    const DroneConfig& config,
    DroneMotion motion,
    Coord destination,
    double desiredDirection)
{
    DroneRuntime commandDrone = physics.getRuntime();
    DroneContext context{
        commandDrone,
        config,
        motion,
        destination,
        desiredDirection};
    executeDroneState(state, context);

    return physics.executeCommandSync({
        motion,
        destination,
        desiredDirection,
        context.drone,
        context.completed,
        nullptr});
}

SimStep advanceOneDynamicStep(
    DronePhysics& physics,
    std::unique_ptr<IDroneState>& state,
    const DroneConfig& config,
    const AttackPlan& plan)
{
    DronePhysicsResult result = executePhysicsCommand(
        physics,
        state,
        config,
        DroneMotion::DYNAMIC,
        plan.dropPlan.firePoint,
        0.0);

    return makeStep(
        result.telemetry,
        plan.targetIndex,
        plan.dropPlan.firePoint,
        plan.horizontalDistance,
        plan.impactTarget);
}

SimStep advanceTowardStopPoint(
    DronePhysics& physics,
    std::unique_ptr<IDroneState>& state,
    const DroneConfig& config,
    int targetIndex,
    Coord destination,
    Coord displayedDropPoint,
    double horizontalDistance,
    Coord predictedTarget)
{
    DronePhysicsResult result = executePhysicsCommand(
        physics,
        state,
        config,
        DroneMotion::STOP_AT_POINT,
        destination,
        0.0);

    return makeStep(
        result.telemetry,
        targetIndex,
        displayedDropPoint,
        horizontalDistance,
        predictedTarget);
}

SimStep advanceTurnInPlace(
    DronePhysics& physics,
    std::unique_ptr<IDroneState>& state,
    const DroneConfig& config,
    const MissionRuntime& mission,
    bool& aligned)
{
    DronePhysicsResult result = executePhysicsCommand(
        physics,
        state,
        config,
        DroneMotion::TURN_IN_PLACE,
        {0.0, 0.0},
        mission.attackDirection);
    aligned = result.completed;

    return makeStep(
        result.telemetry,
        mission.targetIndex,
        mission.firePoint,
        mission.horizontalDistance,
        mission.impactTarget);
}

SimStep advanceLockedAttackRun(
    DronePhysics& physics,
    std::unique_ptr<IDroneState>& state,
    const DroneConfig& config,
    const MissionRuntime& mission,
    bool& reachedFirePoint)
{
    DronePhysicsResult result = executePhysicsCommand(
        physics,
        state,
        config,
        DroneMotion::LOCKED_ATTACK_RUN,
        mission.firePoint,
        mission.attackDirection);
    reachedFirePoint = result.completed;

    return makeStep(
        result.telemetry,
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
    std::unique_ptr<IConfigLoader> loader,
    std::shared_ptr<DronePhysics> physics)
    : targets_(std::move(targets)),
      solver_(std::move(solver)),
      loader_(std::move(loader)),
      physics_(std::move(physics))
{
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::buildPlanForTarget(
    int targetIndex,
    double currentTime,
    const DroneRuntime& drone,
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

    Coord* targetPath = targets_->getTarget(targetIndex);
    const int timeSteps = targets_->getTimeSteps();
    if (targetPath == nullptr || timeSteps <= 0)
    {
        return false;
    }

    BallisticResult ballistic = {};
    if (!solver_->solve(
            loader_->getConfig(),
            loader_->getAmmoParams(),
            ballistic))
    {
        return false;
    }

    plan = model::buildAttackPlanWithBallistics(
        loader_->getConfig(),
        targetPath,
        timeSteps,
        targetIndex,
        drone,
        currentTime,
        ballistic.fallTime,
        ballistic.horizontalDistance);

    return plan.fallTime > model::EPS &&
           std::isfinite(plan.horizontalDistance) &&
           plan.horizontalDistance >= 0.0;
}

int MissionProcessor::chooseBestTargetFromState(
    double currentTime,
    int currentTargetIndex,
    const DroneRuntime& drone,
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
        validPlans[i] = buildPlanForTarget(
            i,
            currentTime,
            drone,
            plans[i]);
    }

    if (currentTargetIndex >= 0 &&
        currentTargetIndex < targetCount &&
        validPlans[currentTargetIndex] &&
        (drone.state == state_code::DECELERATING ||
         drone.state == state_code::TURNING))
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
            drone,
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

bool MissionProcessor::appendStep(const SimStep& step)
{
    std::lock_guard<std::mutex> lock(stepsMutex_);
    if (stepCount_ >= MAX_STEPS)
    {
        finished_ = true;
        return false;
    }
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

    if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr ||
        physics_ == nullptr)
    {
        return;
    }

    const DroneConfig& config = loader_->getConfig();
    physics_->configure(config);
    physics_->reset(config.startPos, config.initialDir);
    state_ = std::make_unique<StateStopped>();

    if (auto* threadSafeTargets =
            dynamic_cast<ThreadSafeTargetProvider*>(targets_.get()))
    {
        threadSafeTargets->setTiming(config.arrayTimeStep, config.timeScale);
    }

    DroneRuntime drone = physics_->getRuntime();
    AttackPlan initialPlan = {};
    int initialBestIndex = chooseBestTargetFromState(
        0.0,
        -1,
        drone,
        initialPlan);
    if (initialBestIndex < 0)
    {
        return;
    }

    Coord initialWaypoint = initialPlan.dropPlan.needManeuver
                                ? initialPlan.dropPlan.maneuverPoint
                                : initialPlan.dropPlan.firePoint;
    double initialDesiredDirection = model::directionToRadians(
        drone.position,
        initialWaypoint,
        drone.direction);
    if (std::fabs(model::calcTurnDeltaRadians(
            drone.direction,
            initialDesiredDirection)) <=
        config.turnThreshold + model::EPS)
    {
        physics_->setDirection(initialDesiredDirection);
        drone.direction = initialDesiredDirection;
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
        drone.position,
        drone.direction,
        drone.state,
        initialPlan.targetIndex,
        initialPlan.dropPlan.firePoint,
        model::calcAimPoint(
            drone.position,
            drone.direction,
            initialPlan.horizontalDistance),
        initialPlan.dropPlan.needManeuver
            ? initialPlan.predictedTarget
            : initialPlan.impactTarget,
        0.0});
}

int MissionProcessor::init()
{
    if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr ||
        physics_ == nullptr)
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
    return initialized_ && !finished_ && stepCount_ < MAX_STEPS &&
           !stopRequested_.load();
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
    DroneRuntime drone = physics_->getRuntime();

    if (mission_.phase == AttackPhase::ATTACK_RUN)
    {
        bool reachedFirePoint = false;
        SimStep next = advanceLockedAttackRun(
            *physics_,
            state_,
            config,
            mission_,
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
            *physics_,
            state_,
            config,
            mission_,
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
            *physics_,
            state_,
            config,
            mission_.targetIndex,
            mission_.maneuverPoint,
            mission_.firePoint,
            mission_.horizontalDistance,
            mission_.impactTarget);
        appendStep(next);

        drone = runtimeFromTelemetry(physics_->getTelemetry());
        if (drone.speed <= model::EPS &&
            model::length(
                drone.position - mission_.maneuverPoint) <= model::EPS)
        {
            mission_.phase = AttackPhase::ALIGN_ATTACK;
        }
        return;
    }

    AttackPlan plan = {};
    int bestIndex = chooseBestTargetFromState(
        currentTime,
        currentTargetIndex_,
        drone,
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
            drone.direction);
        mission_.horizontalDistance = plan.horizontalDistance;

        appendStep(advanceTowardStopPoint(
            *physics_,
            state_,
            config,
            mission_.targetIndex,
            mission_.maneuverPoint,
            mission_.firePoint,
            mission_.horizontalDistance,
            mission_.impactTarget));
        return;
    }

    Coord currentAimPoint = {};
    if (isDropWindowReached(
            config,
            plan,
            drone,
            currentAimPoint))
    {
        double currentError = model::length(
            currentAimPoint - plan.impactTarget);
        DroneRuntime nextDrone = drone;
        auto nextState = state_->clone();
        double currentTelemetryTime = physics_->getTelemetry().timeSecSinceStart;
        SimStep nextStep = simulateOneDynamicStep(
            config,
            plan,
            nextDrone,
            nextState,
            currentTelemetryTime);

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
            auto secondState = nextState->clone();
            SimStep secondStep = nextStep;
            Coord secondImpactTarget = nextImpactTarget;
            Coord secondAimPoint = nextAimPoint;
            DropPlan secondDropPlan = nextDropPlan;

            if (stepCount_ + 1 < MAX_STEPS)
            {
                AttackPlan secondPlan = plan;
                secondPlan.dropPlan = nextDropPlan;
                secondPlan.impactTarget = nextImpactTarget;

                SimStep candidateSecondStep = simulateOneDynamicStep(
                    config,
                    secondPlan,
                    secondDrone,
                    secondState,
                    currentTelemetryTime + config.simTimeStep);
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

            nextStep = advanceOneDynamicStep(*physics_, state_, config, plan);
            nextStep.targetIndex = plan.targetIndex;
            nextStep.dropPoint = nextDropPlan.firePoint;
            nextStep.aimPoint = nextAimPoint;
            nextStep.predictedTarget = nextImpactTarget;
            appendStep(nextStep);

            if (useSecondStep)
            {
                AttackPlan secondPlan = plan;
                secondPlan.dropPlan = nextDropPlan;
                secondPlan.impactTarget = nextImpactTarget;
                secondStep = advanceOneDynamicStep(*physics_, state_, config, secondPlan);
                secondStep.targetIndex = plan.targetIndex;
                secondStep.dropPoint = secondDropPlan.firePoint;
                secondStep.aimPoint = secondAimPoint;
                secondStep.predictedTarget = secondImpactTarget;
                appendStep(secondStep);
            }
        }
        else if (stepCount_ > 0)
        {
            std::lock_guard<std::mutex> lock(stepsMutex_);
            steps_[static_cast<std::size_t>(stepCount_ - 1)].targetIndex =
                plan.targetIndex;
            steps_[static_cast<std::size_t>(stepCount_ - 1)].dropPoint =
                plan.dropPlan.firePoint;
            steps_[static_cast<std::size_t>(stepCount_ - 1)].aimPoint =
                currentAimPoint;
            steps_[static_cast<std::size_t>(stepCount_ - 1)].predictedTarget =
                plan.impactTarget;
        }

        finished_ = true;
        return;
    }

    appendStep(advanceOneDynamicStep(
        *physics_,
        state_,
        config,
        plan));
}

void MissionProcessor::reset()
{
    initializeRuntime();
}

void MissionProcessor::changeSolver(
    std::unique_ptr<IBallisticSolver> solver)
{
    if (solver != nullptr)
    {
        solver_ = std::move(solver);
    }
}

void MissionProcessor::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (hasNext())
    {
        step();
        const DroneConfig& config = loader_->getConfig();
        const double scale = config.timeScale > model::EPS
                                 ? config.timeScale
                                 : 1000.0;
        std::this_thread::sleep_for(
            std::chrono::duration<double>(config.simTimeStep / scale));
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
    return stepCount_;
}
