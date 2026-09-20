#include "model_math.hpp"

#include <algorithm>
#include <cmath>

namespace model
{
double length(Coord value)
{
    return std::sqrt(value.x * value.x + value.y * value.y);
}

Coord normalize(Coord value)
{
    double valueLength = length(value);
    if (valueLength <= EPS)
    {
        return {1.0, 0.0};
    }
    return value / valueLength;
}

double normalizeAngleTwoPi(double angleRadians)
{
    double result = std::fmod(angleRadians, TWO_PI);
    if (result < 0.0)
    {
        result += TWO_PI;
    }
    return result;
}

double calcTurnDeltaRadians(double currentDirection, double desiredDirection)
{
    double delta = normalizeAngleTwoPi(desiredDirection) -
                   normalizeAngleTwoPi(currentDirection);
    while (delta > PI)
    {
        delta -= TWO_PI;
    }
    while (delta <= -PI)
    {
        delta += TWO_PI;
    }
    return delta;
}

double directionToRadians(Coord from, Coord to, double fallbackDirection)
{
    if (from == to)
    {
        return normalizeAngleTwoPi(fallbackDirection);
    }
    return normalizeAngleTwoPi(
        std::atan2(to.y - from.y, to.x - from.x));
}

double calcDroneAcceleration(double attackSpeed, double accelerationPath)
{
    if (attackSpeed <= EPS || accelerationPath <= EPS)
    {
        return 0.0;
    }
    return attackSpeed * attackSpeed / (2.0 * accelerationPath);
}

double estimateTravelTimeFromStopped(
    double distance,
    double attackSpeed,
    double accelerationPath)
{
    if (distance <= EPS || attackSpeed <= EPS)
    {
        return 0.0;
    }

    double acceleration = calcDroneAcceleration(attackSpeed, accelerationPath);
    if (acceleration <= EPS || accelerationPath <= EPS)
    {
        return distance / attackSpeed;
    }

    if (distance <= accelerationPath + EPS)
    {
        return std::sqrt(2.0 * distance / acceleration);
    }

    double accelerationTime = attackSpeed / acceleration;
    double cruiseDistance = distance - accelerationPath;
    return accelerationTime + cruiseDistance / attackSpeed;
}

double estimateTravelTimeWithCurrentSpeed(
    double distance,
    double currentSpeed,
    double attackSpeed,
    double accelerationPath)
{
    if (distance <= EPS)
    {
        return 0.0;
    }
    if (attackSpeed <= EPS)
    {
        return 0.0;
    }

    currentSpeed = std::clamp(currentSpeed, 0.0, attackSpeed);

    double acceleration = calcDroneAcceleration(attackSpeed, accelerationPath);
    if (acceleration <= EPS || accelerationPath <= EPS)
    {
        return distance / attackSpeed;
    }

    if (currentSpeed >= attackSpeed - EPS)
    {
        return distance / attackSpeed;
    }

    double accelDistanceRemaining =
        (attackSpeed * attackSpeed - currentSpeed * currentSpeed) /
        (2.0 * acceleration);
    double accelTimeRemaining =
        (attackSpeed - currentSpeed) / acceleration;

    if (distance <= accelDistanceRemaining + EPS)
    {
        return (-currentSpeed +
                std::sqrt(currentSpeed * currentSpeed +
                          2.0 * acceleration * distance)) /
               acceleration;
    }

    return accelTimeRemaining +
           (distance - accelDistanceRemaining) / attackSpeed;
}

double estimateTimeToPointWithManeuver(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord point)
{
    double distance = length(point - drone.position);
    double desiredDirection = directionToRadians(
        drone.position,
        point,
        drone.direction);
    double turnDelta = std::fabs(calcTurnDeltaRadians(
        drone.direction,
        desiredDirection));

    if (turnDelta <= config.turnThreshold + EPS)
    {
        return estimateTravelTimeWithCurrentSpeed(
            distance,
            drone.speed,
            config.attackSpeed,
            config.accelPath);
    }

    double acceleration = calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);
    double stopTime = 0.0;
    Coord stoppedPosition = drone.position;
    if (acceleration > EPS && drone.speed > EPS)
    {
        double stopDistance =
            drone.speed * drone.speed / (2.0 * acceleration);
        stoppedPosition = drone.position +
                          Coord{std::cos(drone.direction),
                                std::sin(drone.direction)} *
                              stopDistance;
        stopTime = drone.speed / acceleration;
    }

    double desiredDirectionAfterStop = directionToRadians(
        stoppedPosition,
        point,
        drone.direction);
    double turnAfterStop = std::fabs(calcTurnDeltaRadians(
        drone.direction,
        desiredDirectionAfterStop));
    double turnTime = 0.0;
    if (config.angularSpeed > EPS)
    {
        turnTime = turnAfterStop / config.angularSpeed;
    }

    double distanceAfterStop = length(point - stoppedPosition);
    double travelAfterTurn = estimateTravelTimeFromStopped(
        distanceAfterStop,
        config.attackSpeed,
        config.accelPath);

    return stopTime + turnTime + travelAfterTurn;
}

struct StoppedStateEstimate
{
    Coord position;
    double direction;
    double time;
};

StoppedStateEstimate estimateStoppedState(
    const DroneConfig& config,
    const DroneRuntime& drone)
{
    StoppedStateEstimate result = {
        drone.position,
        drone.direction,
        0.0};

    double acceleration = calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);
    if (drone.speed <= EPS || acceleration <= EPS)
    {
        return result;
    }

    double stopDistance =
        drone.speed * drone.speed / (2.0 * acceleration);
    result.position = drone.position +
                      Coord{std::cos(drone.direction),
                            std::sin(drone.direction)} *
                          stopDistance;
    result.time = drone.speed / acceleration;
    return result;
}

double estimateTimeStoppedToStopped(
    double distance,
    double attackSpeed,
    double accelerationPath)
{
    if (distance <= EPS)
    {
        return 0.0;
    }

    double acceleration = calcDroneAcceleration(attackSpeed, accelerationPath);
    if (acceleration <= EPS || attackSpeed <= EPS)
    {
        return attackSpeed > EPS ? distance / attackSpeed : 0.0;
    }

    double fullProfileDistance = 2.0 * accelerationPath;
    if (distance <= fullProfileDistance + EPS)
    {
        return 2.0 * std::sqrt(distance / acceleration);
    }

    double accelerationTime = attackSpeed / acceleration;
    return 2.0 * accelerationTime +
           (distance - fullProfileDistance) / attackSpeed;
}

double estimateTimeMovingToStopped(
    double distance,
    double currentSpeed,
    double attackSpeed,
    double accelerationPath)
{
    if (distance <= EPS)
    {
        return 0.0;
    }

    double acceleration = calcDroneAcceleration(attackSpeed, accelerationPath);
    if (acceleration <= EPS || attackSpeed <= EPS)
    {
        return attackSpeed > EPS ? distance / attackSpeed : 0.0;
    }

    currentSpeed = std::clamp(currentSpeed, 0.0, attackSpeed);
    double stoppingDistance =
        currentSpeed * currentSpeed / (2.0 * acceleration);
    if (distance + EPS < stoppingDistance)
    {
        return -1.0;
    }

    double peakSpeedSquared =
        acceleration * distance + 0.5 * currentSpeed * currentSpeed;
    double peakSpeed = std::sqrt(std::max(0.0, peakSpeedSquared));
    if (peakSpeed <= attackSpeed + EPS)
    {
        return (peakSpeed - currentSpeed) / acceleration +
               peakSpeed / acceleration;
    }

    double accelerationDistance =
        (attackSpeed * attackSpeed - currentSpeed * currentSpeed) /
        (2.0 * acceleration);
    double decelerationDistance =
        attackSpeed * attackSpeed / (2.0 * acceleration);
    double cruiseDistance =
        distance - accelerationDistance - decelerationDistance;
    if (cruiseDistance < 0.0)
    {
        cruiseDistance = 0.0;
    }

    return (attackSpeed - currentSpeed) / acceleration +
           cruiseDistance / attackSpeed +
           attackSpeed / acceleration;
}

struct ManeuverCandidate
{
    Coord point;
    Coord firePoint;
    double attackDirection;
    double timeToRelease;
};

double estimateManeuverCandidateTime(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord impactTarget,
    double radiusToImpact,
    double attackAngle,
    ManeuverCandidate* candidate)
{
    Coord attackVector = {
        std::cos(attackAngle),
        std::sin(attackAngle)};
    Coord maneuverPoint =
        impactTarget - attackVector * radiusToImpact;
    Coord firePoint =
        maneuverPoint + attackVector * config.accelPath;

    double acceleration = calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);
    double attackRunTime = acceleration > EPS
                               ? config.attackSpeed / acceleration
                               : 0.0;

    double arrivalTime = 0.0;
    double arrivalDirection = drone.direction;
    double distanceToManeuver =
        length(maneuverPoint - drone.position);
    double directionToManeuver = directionToRadians(
        drone.position,
        maneuverPoint,
        drone.direction);
    double initialTurn = std::fabs(calcTurnDeltaRadians(
        drone.direction,
        directionToManeuver));
    double stoppingDistance = acceleration > EPS
                                  ? drone.speed * drone.speed /
                                        (2.0 * acceleration)
                                  : 0.0;

    if (initialTurn <= config.turnThreshold + EPS &&
        distanceToManeuver + EPS >= stoppingDistance)
    {
        double movingTime = estimateTimeMovingToStopped(
            distanceToManeuver,
            drone.speed,
            config.attackSpeed,
            config.accelPath);
        if (movingTime >= 0.0)
        {
            arrivalTime = movingTime;
            arrivalDirection = directionToManeuver;
        }
    }

    if (arrivalTime <= EPS && distanceToManeuver > EPS)
    {
        StoppedStateEstimate stopped = estimateStoppedState(config, drone);
        double directionFromStop = directionToRadians(
            stopped.position,
            maneuverPoint,
            stopped.direction);
        double turnToManeuver = std::fabs(calcTurnDeltaRadians(
            stopped.direction,
            directionFromStop));
        double turnTime = config.angularSpeed > EPS
                              ? turnToManeuver / config.angularSpeed
                              : 0.0;

        arrivalTime =
            stopped.time + turnTime +
            estimateTimeStoppedToStopped(
                length(maneuverPoint - stopped.position),
                config.attackSpeed,
                config.accelPath);
        arrivalDirection = directionFromStop;
    }

    double finalTurn = std::fabs(calcTurnDeltaRadians(
        arrivalDirection,
        attackAngle));
    double finalTurnTime = config.angularSpeed > EPS
                               ? finalTurn / config.angularSpeed
                               : 0.0;
    double total = arrivalTime + finalTurnTime + attackRunTime;

    if (candidate != nullptr)
    {
        candidate->point = maneuverPoint;
        candidate->firePoint = firePoint;
        candidate->attackDirection = normalizeAngleTwoPi(attackAngle);
        candidate->timeToRelease = total;
    }

    return total;
}

ManeuverCandidate findBestManeuverCandidate(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord impactTarget,
    double horizontalDistance)
{
    double radiusToImpact = horizontalDistance + config.accelPath;
    double angularResolution = std::max(
        config.turnThreshold,
        config.angularSpeed * config.simTimeStep);
    if (angularResolution <= EPS)
    {
        angularResolution = 0.1;
    }

    int sampleCount = static_cast<int>(
        std::ceil(TWO_PI / angularResolution));
    if (sampleCount < 8)
    {
        sampleCount = 8;
    }

    double step = TWO_PI / static_cast<double>(sampleCount);
    int bestIndex = 0;
    double bestTime = 0.0;
    ManeuverCandidate best = {};

    for (int i = 0; i < sampleCount; ++i)
    {
        double angle = step * static_cast<double>(i);
        ManeuverCandidate candidate = {};
        double time = estimateManeuverCandidateTime(
            config,
            drone,
            impactTarget,
            radiusToImpact,
            angle,
            &candidate);
        if (i == 0 || time < bestTime)
        {
            bestIndex = i;
            bestTime = time;
            best = candidate;
        }
    }

    double left = step * static_cast<double>(bestIndex) - step;
    double right = step * static_cast<double>(bestIndex) + step;
    const double golden = 0.5 * (std::sqrt(5.0) - 1.0);

    double x1 = right - golden * (right - left);
    double x2 = left + golden * (right - left);
    double f1 = estimateManeuverCandidateTime(
        config,
        drone,
        impactTarget,
        radiusToImpact,
        normalizeAngleTwoPi(x1),
        nullptr);
    double f2 = estimateManeuverCandidateTime(
        config,
        drone,
        impactTarget,
        radiusToImpact,
        normalizeAngleTwoPi(x2),
        nullptr);

    for (int iteration = 0; iteration < 32; ++iteration)
    {
        if (f1 <= f2)
        {
            right = x2;
            x2 = x1;
            f2 = f1;
            x1 = right - golden * (right - left);
            f1 = estimateManeuverCandidateTime(
                config,
                drone,
                impactTarget,
                radiusToImpact,
                normalizeAngleTwoPi(x1),
                nullptr);
        }
        else
        {
            left = x1;
            x1 = x2;
            f1 = f2;
            x2 = left + golden * (right - left);
            f2 = estimateManeuverCandidateTime(
                config,
                drone,
                impactTarget,
                radiusToImpact,
                normalizeAngleTwoPi(x2),
                nullptr);
        }
    }

    double refinedAngle = normalizeAngleTwoPi(0.5 * (left + right));
    ManeuverCandidate refined = {};
    double refinedTime = estimateManeuverCandidateTime(
        config,
        drone,
        impactTarget,
        radiusToImpact,
        refinedAngle,
        &refined);
    if (refinedTime < bestTime)
    {
        best = refined;
    }

    return best;
}

DropPlan calculateDynamicDropPlan(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord target,
    double horizontalDistance,
    double& timeToRelease)
{
    Coord toTarget = target - drone.position;
    double distanceToTarget = length(toTarget);
    Coord attackVector = normalize(toTarget);
    Coord firePoint = target - attackVector * horizontalDistance;

    double desiredDirection = directionToRadians(
        drone.position,
        firePoint,
        drone.direction);
    double turnDelta = std::fabs(calcTurnDeltaRadians(
        drone.direction,
        desiredDirection));
    double acceleration = calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    double remainingAccelerationDistance = 0.0;
    if (turnDelta > config.turnThreshold + EPS)
    {
        remainingAccelerationDistance = config.accelPath;
    }
    else if (acceleration > EPS)
    {
        double currentSpeed = std::clamp(
            drone.speed,
            0.0,
            config.attackSpeed);
        remainingAccelerationDistance =
            (config.attackSpeed * config.attackSpeed -
             currentSpeed * currentSpeed) /
            (2.0 * acceleration);
    }

    DropPlan plan = {};
    if (horizontalDistance + remainingAccelerationDistance <=
        distanceToTarget + EPS)
    {
        plan.needManeuver = false;
        plan.maneuverPoint = drone.position;
        plan.firePoint = firePoint;
        timeToRelease = estimateTimeToPointWithManeuver(
            config,
            drone,
            firePoint);
        return plan;
    }

    ManeuverCandidate candidate = findBestManeuverCandidate(
        config,
        drone,
        target,
        horizontalDistance);
    plan.needManeuver = true;
    plan.maneuverPoint = candidate.point;
    plan.firePoint = candidate.firePoint;
    timeToRelease = candidate.timeToRelease;
    return plan;
}

Coord directionVector(double directionRadians)
{
    return {std::cos(directionRadians), std::sin(directionRadians)};
}

Coord calcAimPoint(
    Coord position,
    double directionRadians,
    double horizontalDistance)
{
    return position +
           directionVector(directionRadians) * horizontalDistance;
}

ObservedTargetState observeTargetFromPast(
    const Coord* targetPath,
    int timeSteps,
    double time,
    double arrayTimeStep)
{
    ObservedTargetState state = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}};
    if (targetPath == nullptr || timeSteps <= 0 || arrayTimeStep <= EPS)
    {
        return state;
    }
    if (time < 0.0)
    {
        time = 0.0;
    }

    long long absoluteIndex = static_cast<long long>(
        std::floor(time / arrayTimeStep + EPS));
    int latestIndex = static_cast<int>(absoluteIndex % timeSteps);
    if (latestIndex < 0)
    {
        latestIndex += timeSteps;
    }
    state.position = targetPath[latestIndex];

    if (absoluteIndex >= 1)
    {
        int previousIndex = latestIndex - 1;
        if (previousIndex < 0)
        {
            previousIndex += timeSteps;
        }
        state.velocity =
            (targetPath[latestIndex] - targetPath[previousIndex]) /
            arrayTimeStep;

        if (absoluteIndex >= 2)
        {
            int beforePreviousIndex = previousIndex - 1;
            if (beforePreviousIndex < 0)
            {
                beforePreviousIndex += timeSteps;
            }

            Coord pk = targetPath[latestIndex];
            Coord pk1 = targetPath[previousIndex];
            Coord pk2 = targetPath[beforePreviousIndex];

            state.velocity =
                (pk * 3.0 - pk1 * 4.0 + pk2) /
                (2.0 * arrayTimeStep);
            state.acceleration =
                (pk - pk1 * 2.0 + pk2) /
                (arrayTimeStep * arrayTimeStep);
        }
    }

    double latestSampleTime = absoluteIndex * arrayTimeStep;
    double elapsed = time - latestSampleTime;
    if (elapsed < 0.0)
    {
        elapsed = 0.0;
    }
    if (elapsed > arrayTimeStep)
    {
        elapsed = arrayTimeStep;
    }

    state.position = state.position + state.velocity * elapsed +
                     state.acceleration * (0.5 * elapsed * elapsed);
    state.velocity = state.velocity + state.acceleration * elapsed;
    return state;
}

Coord predictObservedTarget(
    const ObservedTargetState& state,
    double horizon)
{
    if (horizon < 0.0)
    {
        horizon = 0.0;
    }
    return state.position + state.velocity * horizon +
           state.acceleration * (0.5 * horizon * horizon);
}

Coord predictObservedTargetClamped(
    const ObservedTargetState& state,
    double horizon,
    double accelerationHorizon)
{
    if (horizon < 0.0)
    {
        horizon = 0.0;
    }
    if (accelerationHorizon < 0.0)
    {
        accelerationHorizon = 0.0;
    }

    double curvedTime = horizon;
    if (curvedTime > accelerationHorizon)
    {
        curvedTime = accelerationHorizon;
    }
    Coord position = predictObservedTarget(state, curvedTime);
    if (horizon <= curvedTime + EPS)
    {
        return position;
    }

    Coord velocityAtClamp =
        state.velocity + state.acceleration * curvedTime;
    return position + velocityAtClamp * (horizon - curvedTime);
}

Coord estimatePredictionResidual(
    const Coord* targetPath,
    int timeSteps,
    double currentTime,
    double arrayTimeStep,
    double horizon)
{
    if (horizon <= EPS ||
        currentTime <= horizon + 2.0 * arrayTimeStep)
    {
        return {0.0, 0.0};
    }

    double validationTime = currentTime - horizon;
    ObservedTargetState past = observeTargetFromPast(
        targetPath,
        timeSteps,
        validationTime,
        arrayTimeStep);
    Coord oldPrediction = predictObservedTarget(past, horizon);
    ObservedTargetState current = observeTargetFromPast(
        targetPath,
        timeSteps,
        currentTime,
        arrayTimeStep);
    return current.position - oldPrediction;
}

double solveFallTime(
    const AmmoParams& ammo,
    double altitude,
    double attackSpeed)
{
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    double v0 = attackSpeed;
    double z0 = altitude;

    double a = d * G * m - 2.0 * d * d * l * v0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * v0;
    double c = 6.0 * m * m * z0;

    if (std::fabs(a) <= EPS)
    {
        if (std::fabs(b) <= EPS || c / b >= 0.0)
        {
            return -1.0;
        }
        return std::sqrt(-c / b);
    }

    double p = -(b * b) / (3.0 * a * a);
    double q =
        (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;
    if (p >= 0.0)
    {
        return -1.0;
    }

    double acosArg =
        (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
    acosArg = std::clamp(acosArg, -1.0, 1.0);
    double phi = std::acos(acosArg);
    double bestTime = -1.0;

    for (int k = 0; k < 3; ++k)
    {
        double time =
            2.0 * std::sqrt(-p / 3.0) *
                std::cos((phi + 2.0 * PI * k) / 3.0) -
            b / (3.0 * a);
        if (time > EPS &&
            (bestTime < 0.0 || time < bestTime))
        {
            bestTime = time;
        }
    }

    return bestTime;
}

double calcHorizontalDistance(
    const AmmoParams& ammo,
    double fallTime,
    double attackSpeed)
{
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    double v0 = attackSpeed;
    double t = fallTime;

    double l2 = l * l;
    double l3 = l2 * l;
    double l4 = l2 * l2;
    double d2 = d * d;
    double d3 = d2 * d;
    double d4 = d2 * d2;
    double m2 = m * m;
    double m3 = m2 * m;
    double m4 = m2 * m2;

    double term1 = v0 * t;
    double term2 = -(t * t * d * v0) / (2.0 * m);
    double term3 =
        (t * t * t *
         (6.0 * d * G * l * m -
          6.0 * d2 * (l2 - 1.0) * v0)) /
        (36.0 * m2);
    double term4 =
        (std::pow(t, 4.0) *
         (-6.0 * d2 * G * l * (1.0 + l2 + l4) * m +
          3.0 * d3 * l2 * (1.0 + l2) * v0 +
          6.0 * d3 * l4 * (1.0 + l2) * v0)) /
        (36.0 * (1.0 + l2) * (1.0 + l2) * m3);
    double term5 =
        (std::pow(t, 5.0) *
         (3.0 * d3 * G * l3 * m -
          3.0 * d4 * l2 * (1.0 + l2) * v0)) /
        (36.0 * (1.0 + l2) * m4);

    return term1 + term2 + term3 + term4 + term5;
}

AttackPlan buildAttackPlanWithBallistics(
    const DroneConfig& config,
    const Coord* targetPath,
    int timeSteps,
    int targetIndex,
    const DroneRuntime& drone,
    double currentTime,
    double fallTime,
    double horizontalDistance)
{
    AttackPlan plan = {};
    plan.targetIndex = targetIndex;
    plan.fallTime = fallTime;
    plan.horizontalDistance = horizontalDistance;

    ObservedTargetState observed = observeTargetFromPast(
        targetPath,
        timeSteps,
        currentTime,
        config.arrayTimeStep);
    plan.targetNow = observed.position;
    plan.targetVelocity = observed.velocity;

    double provisionalReleaseTime = 0.0;
    (void)calculateDynamicDropPlan(
        config,
        drone,
        plan.targetNow,
        plan.horizontalDistance,
        provisionalReleaseTime);
    plan.totalTime = provisionalReleaseTime + plan.fallTime;

    Coord predictionResidual = estimatePredictionResidual(
        targetPath,
        timeSteps,
        currentTime,
        config.arrayTimeStep,
        plan.fallTime);
    plan.predictionUncertainty = length(predictionResidual);

    plan.predictedTarget = predictObservedTargetClamped(
        observed,
        plan.totalTime,
        plan.fallTime);
    plan.impactTarget = predictObservedTarget(
                            observed,
                            plan.fallTime) +
                        predictionResidual;

    plan.dropPlan = calculateDynamicDropPlan(
        config,
        drone,
        plan.impactTarget,
        plan.horizontalDistance,
        plan.timeToDrop);

    if (plan.dropPlan.needManeuver)
    {
        ManeuverCandidate candidate = findBestManeuverCandidate(
            config,
            drone,
            plan.predictedTarget,
            plan.horizontalDistance);

        double refinedTotalTime =
            candidate.timeToRelease + plan.fallTime;
        plan.predictedTarget = predictObservedTargetClamped(
            observed,
            refinedTotalTime,
            plan.fallTime);
        candidate = findBestManeuverCandidate(
            config,
            drone,
            plan.predictedTarget,
            plan.horizontalDistance);

        plan.dropPlan.needManeuver = true;
        plan.dropPlan.maneuverPoint = candidate.point;
        plan.dropPlan.firePoint = candidate.firePoint;
        plan.timeToDrop = candidate.timeToRelease;
    }

    plan.totalTime = plan.timeToDrop + plan.fallTime;
    return plan;
}
}

