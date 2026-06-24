#include "analytical_solver.hpp"

#include <cmath>

const double G = 9.81;
const double PI = acos(-1.0);

const double EPS = 1e-9;

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
