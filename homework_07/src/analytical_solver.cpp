#include "analytical_solver.hpp"

#include <cmath>

const double G = 9.81;
const double PI = acos(-1.0);
const double TWO_PI = 2.0 * PI;

const double EPS = 1e-9;

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

double calcTurnDeltaRadians(
    double currentDirection,
    double desiredDirection)
{
    double delta =
        normalizeAngleTwoPi(desiredDirection) -
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

double directionToRadians(
    Coord from,
    Coord to,
    double fallbackDirection)
{
    if (from == to)
    {
        return normalizeAngleTwoPi(fallbackDirection);
    }

    return normalizeAngleTwoPi(
        std::atan2(to.y - from.y, to.x - from.x));
}

double calcDroneAcceleration(
    double attackSpeed,
    double accelerationPath)
{
    if (attackSpeed <= EPS || accelerationPath <= EPS)
    {
        return 0.0;
    }

    return attackSpeed * attackSpeed /
           (2.0 * accelerationPath);
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

    double acceleration =
        calcDroneAcceleration(attackSpeed, accelerationPath);

    if (acceleration <= EPS || accelerationPath <= EPS)
    {
        return distance / attackSpeed;
    }

    if (distance <= accelerationPath + EPS)
    {
        return std::sqrt(2.0 * distance / acceleration);
    }

    double accelerationTime = attackSpeed / acceleration;

    return accelerationTime +
           (distance - accelerationPath) / attackSpeed;
}

double estimateTravelTimeWithCurrentSpeed(
    double distance,
    double currentSpeed,
    double attackSpeed,
    double accelerationPath)
{
    if (distance <= EPS || attackSpeed <= EPS)
    {
        return 0.0;
    }

    if (currentSpeed < 0.0)
    {
        currentSpeed = 0.0;
    }

    if (currentSpeed > attackSpeed)
    {
        currentSpeed = attackSpeed;
    }

    double acceleration =
        calcDroneAcceleration(attackSpeed, accelerationPath);

    if (acceleration <= EPS || accelerationPath <= EPS)
    {
        return distance / attackSpeed;
    }

    if (currentSpeed >= attackSpeed - EPS)
    {
        return distance / attackSpeed;
    }

    double remainingAccelerationDistance =
        (attackSpeed * attackSpeed -
         currentSpeed * currentSpeed) /
        (2.0 * acceleration);

    double remainingAccelerationTime =
        (attackSpeed - currentSpeed) / acceleration;

    if (distance <= remainingAccelerationDistance + EPS)
    {
        return (-currentSpeed +
                std::sqrt(
                    currentSpeed * currentSpeed +
                    2.0 * acceleration * distance)) /
               acceleration;
    }

    return remainingAccelerationTime +
           (distance - remainingAccelerationDistance) /
               attackSpeed;
}

Coord directionVector(double directionRadians)
{
    return {
        std::cos(directionRadians),
        std::sin(directionRadians)
    };
}

Coord calcAimPoint(
    Coord position,
    double directionRadians,
    double horizontalDistance)
{
    return position +
           directionVector(directionRadians) *
               horizontalDistance;
}

double estimateTimeToPointWithManeuver(
    const DroneConfig& config,
    const DroneRuntime& drone,
    Coord point)
{
    double distance = length(point - drone.position);
    double desiredDirection =
        directionToRadians(drone.position, point, drone.direction);
    double turnDelta =
        std::fabs(calcTurnDeltaRadians(
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

    double acceleration =
        calcDroneAcceleration(
            config.attackSpeed,
            config.accelPath);

    double stopTime = 0.0;
    Coord stoppedPosition = drone.position;

    if (acceleration > EPS && drone.speed > EPS)
    {
        double stopDistance =
            drone.speed * drone.speed /
            (2.0 * acceleration);

        stoppedPosition =
            drone.position +
            directionVector(drone.direction) * stopDistance;

        stopTime = drone.speed / acceleration;
    }

    double desiredDirectionAfterStop =
        directionToRadians(
            stoppedPosition,
            point,
            drone.direction);

    double turnAfterStop =
        std::fabs(calcTurnDeltaRadians(
            drone.direction,
            desiredDirectionAfterStop));

    double turnTime = 0.0;

    if (config.angularSpeed > EPS)
    {
        turnTime = turnAfterStop / config.angularSpeed;
    }

    double distanceAfterStop =
        length(point - stoppedPosition);

    double travelAfterTurn =
        estimateTravelTimeFromStopped(
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
        0.0
    };

    double acceleration = calcDroneAcceleration(
        config.attackSpeed,
        config.accelPath);

    if (drone.speed <= EPS || acceleration <= EPS)
    {
        return result;
    }

    double stopDistance =
        drone.speed * drone.speed /
        (2.0 * acceleration);

    result.position =
        drone.position +
        directionVector(drone.direction) * stopDistance;

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

    double acceleration =
        calcDroneAcceleration(attackSpeed, accelerationPath);

    if (acceleration <= EPS || attackSpeed <= EPS)
    {
        return attackSpeed > EPS
                   ? distance / attackSpeed
                   : 0.0;
    }

    double fullProfileDistance =
        2.0 * accelerationPath;

    if (distance <= fullProfileDistance + EPS)
    {
        return 2.0 * std::sqrt(distance / acceleration);
    }

    double accelerationTime =
        attackSpeed / acceleration;

    return 2.0 * accelerationTime +
           (distance - fullProfileDistance) / attackSpeed;
}
struct ObservedTargetState
{
    Coord position;
    Coord velocity;
    Coord acceleration;
};

ObservedTargetState observeTargetFromPast(
    const Coord* targetPath,
    int timeSteps,
    double time,
    double arrayTimeStep)
{
    ObservedTargetState state = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}
    };

    if (targetPath == nullptr || timeSteps <= 0 || arrayTimeStep <= EPS)
    {
        return state;
    }

    if (time < 0.0)
    {
        time = 0.0;
    }

    long long absoluteIndex =
        static_cast<long long>(std::floor(time / arrayTimeStep + EPS));

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

    state.position =
        state.position +
        state.velocity * elapsed +
        state.acceleration * (0.5 * elapsed * elapsed);

    state.velocity =
        state.velocity + state.acceleration * elapsed;

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

    return state.position +
           state.velocity * horizon +
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

    return position +
           velocityAtClamp * (horizon - curvedTime);
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

    Coord oldPrediction =
        predictObservedTarget(past, horizon);

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
        (2.0 * b * b * b) / (27.0 * a * a * a) +
        c / a;

    if (p >= 0.0)
    {
        return -1.0;
    }

    double acosArg =
        (3.0 * q / (2.0 * p)) *
        std::sqrt(-3.0 / p);

    if (acosArg < -1.0)
    {
        acosArg = -1.0;
    }

    if (acosArg > 1.0)
    {
        acosArg = 1.0;
    }

    double phi = std::acos(acosArg);
    double bestTime = -1.0;

    for (int k = 0; k < 3; ++k)
    {
        double t =
            2.0 * std::sqrt(-p / 3.0) *
                std::cos((phi + 2.0 * PI * k) / 3.0) -
            b / (3.0 * a);

        if (t > EPS &&
            (bestTime < 0.0 || t < bestTime))
        {
            bestTime = t;
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
    double term2 =
        -(t * t * d * v0) / (2.0 * m);

    double term3 =
        (t * t * t *
         (6.0 * d * G * l * m -
          6.0 * d2 * (l2 - 1.0) * v0)) /
        (36.0 * m2);

    double term4 =
        (std::pow(t, 4.0) *
         (-6.0 * d2 * G * l *
              (1.0 + l2 + l4) * m +
          3.0 * d3 * l2 *
              (1.0 + l2) * v0 +
          6.0 * d3 * l4 *
              (1.0 + l2) * v0)) /
        (36.0 *
         (1.0 + l2) *
         (1.0 + l2) *
         m3);

    double term5 =
        (std::pow(t, 5.0) *
         (3.0 * d3 * G * l3 * m -
          3.0 * d4 * l2 *
              (1.0 + l2) * v0)) /
        (36.0 * (1.0 + l2) * m4);

    return term1 + term2 + term3 + term4 + term5;
}

double calculateD(double droneX, double droneY, double targetX, double targetY)
{
    double dx = targetX - droneX;
    double dy = targetY - droneY;
    return sqrt(dx * dx + dy * dy);
}

double calculateH(const InputData& data, double t)
{
    double V0 = data.attackSpeed;
    double d = data.d;
    double m = data.m;
    double l = data.l;

    double h =
        V0 * t
        - (t * t * d * V0) / (2 * m)
        + (pow(t, 3) * (6 * d * G * l * m - 6 * d * d * (l * l - 1) * V0)) / (36 * m * m)
        + (pow(t, 4) * (-6 * d * d * G * l * (1 + l * l + l * l * l * l) * m
        + 3 * d * d * d * l * l * (1 + l * l) * V0
        + 6 * d * d * d * l * l * l * l * (1 + l * l) * V0))
        / (36 * pow(1 + l * l, 2) * pow(m, 3))
        + (pow(t, 5) * (3 * d * d * d * G * l * l * l * m
        - 3 * pow(d, 4) * l * l * (1 + l * l) * V0))
        / (36 * (1 + l * l) * pow(m, 4));

    return h;
}

double calculateT(const InputData& data)
{
    double a = data.d * G * data.m - 2 * data.d * data.d * data.l * data.attackSpeed;
    double b = -3 * G * data.m * data.m + 3 * data.d * data.l * data.m * data.attackSpeed;
    double c = 6 * data.m * data.m * data.zd;

    double p = -(b * b) / (3 * a * a);
    if (p >= 0) return -1;

    double q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    double acosArg = (3 * q / (2 * p)) * sqrt(-3 / p);
    if (acosArg < -1 || acosArg > 1) return -1;

    double phi = acos(acosArg);

    double t = 2 * sqrt(-p / 3) * cos((phi + 4 * PI) / 3) - b / (3 * a);

    if (t <= 0) return -1;

    return t;
}

bool calculateAttackPlan(
    InputData& data,
    double droneX,
    double droneY,
    double targetX,
    double targetY,
    double& aimX,
    double& aimY,
    double& fireX,
    double& fireY,
    double& totalTime,
    bool& needManeuver)
{
    double D = calculateD(droneX, droneY, targetX, targetY);
    if (D <= 0.0)
        return false;

    data.targetX = targetX;
    data.targetY = targetY;

    double tFall = calculateT(data);
    if (tFall < 0.0)
        return false;

    double h = calculateH(data, tFall);
    if (h <= 1e-6)
        return false;

    double acceleration =
        data.attackSpeed * data.attackSpeed / (2.0 * data.accelerationPath);

    auto timeFromStop = [&](double dist) -> double
    {
        if (dist <= 0.0)
            return 0.0;

        if (dist <= data.accelerationPath)
        {
            return sqrt(2.0 * dist / acceleration);
        }
        else
        {
            double accelTime = data.attackSpeed / acceleration;
            return accelTime + (dist - data.accelerationPath) / data.attackSpeed;
        }
    };

    if (D < h + data.accelerationPath)
    {
        needManeuver = true;

        aimX = targetX - (targetX - droneX) * (h + data.accelerationPath) / D;
        aimY = targetY - (targetY - droneY) * (h + data.accelerationPath) / D;

        double D2 = calculateD(aimX, aimY, targetX, targetY);
        if (D2 <= 0.0)
            return false;

        double k = (D2 - h) / D2;

        fireX = aimX + (targetX - aimX) * k;
        fireY = aimY + (targetY - aimY) * k;

        double distToAim = calculateD(droneX, droneY, aimX, aimY);
        double distAimToFire = calculateD(aimX, aimY, fireX, fireY);

        totalTime =
            distToAim / data.attackSpeed +
            timeFromStop(distAimToFire) +
            tFall;
    }
    else
    {
        needManeuver = false;

        double k = (D - h) / D;

        fireX = droneX + (targetX - droneX) * k;
        fireY = droneY + (targetY - droneY) * k;

        aimX = fireX;
        aimY = fireY;

        double distToFire = calculateD(droneX, droneY, fireX, fireY);
        totalTime = distToFire / data.attackSpeed + tFall;
    }

    return true;
}

bool AnalyticalSolver::solve(
    InputData& data,
    double droneX,
    double droneY,
    double targetX,
    double targetY,
    double& aimX,
    double& aimY,
    double& fireX,
    double& fireY,
    double& totalTime,
    bool& needManeuver)
{
    return calculateAttackPlan(
        data,
        droneX,
        droneY,
        targetX,
        targetY,
        aimX,
        aimY,
        fireX,
        fireY,
        totalTime,
        needManeuver);
}
