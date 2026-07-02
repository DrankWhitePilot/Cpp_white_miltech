#include "drone_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "model_math.hpp"

namespace
{
bool speedIncreased(const DroneContext& ctx)
{
    return ctx.drone.speed > ctx.previousSpeed + model::EPS;
}

bool speedDecreased(const DroneContext& ctx)
{
    return ctx.drone.speed + model::EPS < ctx.previousSpeed;
}

bool isStopped(const DroneContext& ctx)
{
    return ctx.drone.speed <= model::EPS;
}

void performDynamicMotion(DroneContext& ctx)
{
    const double dt = ctx.config.simTimeStep;
    const double acceleration = model::calcDroneAcceleration(
        ctx.config.attackSpeed,
        ctx.config.accelPath);

    const double desiredDirection = model::directionToRadians(
        ctx.drone.position,
        ctx.destination,
        ctx.drone.direction);
    const double turnDelta = model::calcTurnDeltaRadians(
        ctx.drone.direction,
        desiredDirection);

    ctx.turnRequired =
        std::fabs(turnDelta) >
        ctx.config.turnThreshold + model::EPS;
    ctx.turnCompleted = false;

    if (ctx.turnRequired)
    {
        if (ctx.drone.speed > model::EPS &&
            acceleration > model::EPS)
        {
            const double oldSpeed = ctx.drone.speed;
            double newSpeed = oldSpeed - acceleration * dt;
            if (newSpeed < 0.0)
            {
                newSpeed = 0.0;
            }

            const double moveDistance =
                0.5 * (oldSpeed + newSpeed) * dt;
            ctx.drone.position =
                ctx.drone.position +
                model::directionVector(ctx.drone.direction) *
                    moveDistance;
            ctx.drone.speed = newSpeed;
            return;
        }

        ctx.drone.speed = 0.0;
        const double maxTurn = ctx.config.angularSpeed * dt;
        ctx.turnCompleted =
            std::fabs(turnDelta) <= maxTurn + model::EPS;
        if (ctx.turnCompleted)
        {
            ctx.drone.direction = desiredDirection;
        }
        else
        {
            ctx.drone.direction = model::normalizeAngleTwoPi(
                ctx.drone.direction +
                (turnDelta > 0.0 ? maxTurn : -maxTurn));
        }
        return;
    }

    const Coord oldVelocity =
        model::directionVector(ctx.drone.direction) *
        ctx.drone.speed;
    const Coord desiredVelocity =
        model::directionVector(desiredDirection) *
        ctx.config.attackSpeed;
    const Coord deltaVelocity = desiredVelocity - oldVelocity;
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
            newVelocity =
                oldVelocity +
                deltaVelocity *
                    (maxDeltaVelocity / deltaVelocityLength);
        }
    }

    const double newSpeed = model::length(newVelocity);
    const Coord averageVelocity =
        (oldVelocity + newVelocity) * 0.5;
    ctx.drone.position =
        ctx.drone.position + averageVelocity * dt;

    if (newSpeed > model::EPS)
    {
        ctx.drone.direction = model::normalizeAngleTwoPi(
            std::atan2(newVelocity.y, newVelocity.x));
    }

    ctx.drone.speed = newSpeed;
}

void performStopAtPointMotion(DroneContext& ctx)
{
    const double dt = ctx.config.simTimeStep;
    const double acceleration = model::calcDroneAcceleration(
        ctx.config.attackSpeed,
        ctx.config.accelPath);
    const Coord toDestination =
        ctx.destination - ctx.drone.position;
    const double remaining = model::length(toDestination);

    ctx.turnRequired = false;
    ctx.turnCompleted = false;

    if (remaining <= model::EPS &&
        ctx.drone.speed <= model::EPS)
    {
        ctx.drone.position = ctx.destination;
        ctx.drone.speed = 0.0;
        ctx.completed = true;
        return;
    }

    const double desiredDirection = model::directionToRadians(
        ctx.drone.position,
        ctx.destination,
        ctx.drone.direction);
    const double turnDelta = model::calcTurnDeltaRadians(
        ctx.drone.direction,
        desiredDirection);

    ctx.turnRequired = std::fabs(turnDelta) > model::EPS;
    if (ctx.turnRequired)
    {
        if (ctx.drone.speed > model::EPS &&
            acceleration > model::EPS)
        {
            const double oldSpeed = ctx.drone.speed;
            const double newSpeed = std::max(
                0.0,
                oldSpeed - acceleration * dt);
            const double distance =
                0.5 * (oldSpeed + newSpeed) * dt;
            ctx.drone.position =
                ctx.drone.position +
                model::directionVector(ctx.drone.direction) *
                    distance;
            ctx.drone.speed = newSpeed;
            return;
        }

        ctx.drone.speed = 0.0;
        const double maxTurn = ctx.config.angularSpeed * dt;
        ctx.turnCompleted =
            std::fabs(turnDelta) <= maxTurn + model::EPS;
        if (ctx.turnCompleted)
        {
            ctx.drone.direction = desiredDirection;
        }
        else
        {
            ctx.drone.direction = model::normalizeAngleTwoPi(
                ctx.drone.direction +
                (turnDelta > 0.0 ? maxTurn : -maxTurn));
        }
        return;
    }

    if (acceleration > model::EPS &&
        ctx.drone.speed <= acceleration * dt + model::EPS)
    {
        const double stoppingDistance =
            ctx.drone.speed * ctx.drone.speed /
            (2.0 * acceleration);
        if (std::fabs(stoppingDistance - remaining) <= 1e-7)
        {
            ctx.drone.position = ctx.destination;
            ctx.drone.speed = 0.0;
            ctx.completed = true;
            return;
        }
    }

    const double oldSpeed = ctx.drone.speed;
    const double minimumNextSpeed = std::max(
        0.0,
        oldSpeed - acceleration * dt);
    const double maximumNextSpeed = std::min(
        ctx.config.attackSpeed,
        oldSpeed + acceleration * dt);

    const double discriminant =
        acceleration * acceleration * dt * dt -
        4.0 * (acceleration * dt * oldSpeed -
               2.0 * acceleration * remaining);
    double stopLimitedSpeed = 0.0;
    if (discriminant > 0.0)
    {
        stopLimitedSpeed =
            (-acceleration * dt + std::sqrt(discriminant)) /
            2.0;
    }
    stopLimitedSpeed = std::max(0.0, stopLimitedSpeed);

    double newSpeed = std::min(
        maximumNextSpeed,
        stopLimitedSpeed);
    if (newSpeed < minimumNextSpeed)
    {
        newSpeed = minimumNextSpeed;
    }

    double moveDistance =
        0.5 * (oldSpeed + newSpeed) * dt;
    if (moveDistance > remaining &&
        moveDistance - remaining <= 1e-7)
    {
        moveDistance = remaining;
    }

    ctx.drone.position =
        ctx.drone.position +
        model::directionVector(desiredDirection) *
            moveDistance;
    ctx.drone.direction = desiredDirection;
    ctx.drone.speed = newSpeed;

    ctx.completed =
        ctx.drone.speed <= model::EPS &&
        model::length(
            ctx.drone.position - ctx.destination) <= model::EPS;
}

void performTurnInPlaceMotion(DroneContext& ctx)
{
    ctx.drone.speed = 0.0;
    const double delta = model::calcTurnDeltaRadians(
        ctx.drone.direction,
        ctx.desiredDirection);
    ctx.turnRequired = std::fabs(delta) > model::EPS;

    const double maxTurn =
        ctx.config.angularSpeed * ctx.config.simTimeStep;
    ctx.turnCompleted =
        std::fabs(delta) <= maxTurn + model::EPS;

    if (ctx.turnCompleted)
    {
        ctx.drone.direction = ctx.desiredDirection;
        ctx.completed = true;
    }
    else
    {
        ctx.drone.direction = model::normalizeAngleTwoPi(
            ctx.drone.direction +
            (delta > 0.0 ? maxTurn : -maxTurn));
    }
}

void performLockedAttackRunMotion(DroneContext& ctx)
{
    const double acceleration = model::calcDroneAcceleration(
        ctx.config.attackSpeed,
        ctx.config.accelPath);
    const double dt = ctx.config.simTimeStep;
    const Coord attackVector =
        model::directionVector(ctx.desiredDirection);
    const double remaining = model::length(
        ctx.destination - ctx.drone.position);
    const double oldSpeed = ctx.drone.speed;
    const double newSpeed = std::min(
        ctx.config.attackSpeed,
        oldSpeed + acceleration * dt);
    const double moveDistance =
        0.5 * (oldSpeed + newSpeed) * dt;

    ctx.turnRequired = false;
    ctx.turnCompleted = false;

    if (moveDistance + model::EPS >= remaining)
    {
        ctx.drone.position = ctx.destination;
        const double reachableSpeed = std::sqrt(std::max(
            0.0,
            oldSpeed * oldSpeed +
                2.0 * acceleration * remaining));
        ctx.drone.speed = std::min(
            ctx.config.attackSpeed,
            reachableSpeed);
        ctx.drone.direction = ctx.desiredDirection;
        ctx.completed = true;
        return;
    }

    ctx.drone.position =
        ctx.drone.position + attackVector * moveDistance;
    ctx.drone.speed = newSpeed;
    ctx.drone.direction = ctx.desiredDirection;
}

void performMotion(DroneContext& ctx)
{
    ctx.completed = false;
    ctx.previousSpeed = ctx.drone.speed;
    ctx.turnRequired = false;
    ctx.turnCompleted = false;

    if (ctx.motion == DroneMotion::DYNAMIC)
    {
        performDynamicMotion(ctx);
        return;
    }
    if (ctx.motion == DroneMotion::STOP_AT_POINT)
    {
        performStopAtPointMotion(ctx);
        return;
    }
    if (ctx.motion == DroneMotion::TURN_IN_PLACE)
    {
        performTurnInPlaceMotion(ctx);
        return;
    }
    performLockedAttackRunMotion(ctx);
}
}

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx)
{
    performMotion(ctx);
    if (ctx.turnRequired)
    {
        return std::make_unique<StateTurning>();
    }
    if (speedIncreased(ctx) || !isStopped(ctx))
    {
        return std::make_unique<StateAccelerating>();
    }
    return nullptr;
}

std::unique_ptr<IDroneState> StateStopped::clone() const
{
    return std::make_unique<StateStopped>();
}

const char* StateStopped::name() const
{
    return "Stopped";
}

int StateStopped::code() const
{
    return state_code::STOPPED;
}

std::unique_ptr<IDroneState> StateAccelerating::execute(
    DroneContext& ctx)
{
    performMotion(ctx);
    if (ctx.turnRequired || speedDecreased(ctx))
    {
        return std::make_unique<StateDecelerating>();
    }
    if (isStopped(ctx))
    {
        return std::make_unique<StateStopped>();
    }
    if (!speedIncreased(ctx) &&
        ctx.drone.speed >=
            ctx.config.attackSpeed - model::EPS)
    {
        return std::make_unique<StateMoving>();
    }
    return nullptr;
}

std::unique_ptr<IDroneState> StateAccelerating::clone() const
{
    return std::make_unique<StateAccelerating>();
}

const char* StateAccelerating::name() const
{
    return "Accelerating";
}

int StateAccelerating::code() const
{
    return state_code::ACCELERATING;
}

std::unique_ptr<IDroneState> StateDecelerating::execute(
    DroneContext& ctx)
{
    performMotion(ctx);

    if (ctx.motion == DroneMotion::DYNAMIC && ctx.turnRequired)
    {
        const double acceleration = model::calcDroneAcceleration(
            ctx.config.attackSpeed,
            ctx.config.accelPath);
        if (ctx.previousSpeed > model::EPS && acceleration > model::EPS)
        {
            return nullptr;
        }
        return std::make_unique<StateTurning>();
    }

    if (isStopped(ctx))
    {
        return std::make_unique<StateStopped>();
    }
    if (speedIncreased(ctx))
    {
        return std::make_unique<StateAccelerating>();
    }
    if (!ctx.turnRequired && !speedDecreased(ctx))
    {
        return std::make_unique<StateMoving>();
    }
    return nullptr;
}

std::unique_ptr<IDroneState> StateDecelerating::clone() const
{
    return std::make_unique<StateDecelerating>();
}

const char* StateDecelerating::name() const
{
    return "Decelerating";
}

int StateDecelerating::code() const
{
    return state_code::DECELERATING;
}

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx)
{
    performMotion(ctx);
    if (ctx.turnRequired && !ctx.turnCompleted)
    {
        return nullptr;
    }
    if (speedDecreased(ctx))
    {
        return std::make_unique<StateDecelerating>();
    }
    if (ctx.turnCompleted || isStopped(ctx) || speedIncreased(ctx))
    {
        return std::make_unique<StateAccelerating>();
    }
    return std::make_unique<StateMoving>();
}

std::unique_ptr<IDroneState> StateTurning::clone() const
{
    return std::make_unique<StateTurning>();
}

const char* StateTurning::name() const
{
    return "Turning";
}

int StateTurning::code() const
{
    return state_code::TURNING;
}

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx)
{
    performMotion(ctx);
    if (ctx.turnRequired || speedDecreased(ctx))
    {
        return std::make_unique<StateDecelerating>();
    }
    if (isStopped(ctx))
    {
        return std::make_unique<StateStopped>();
    }
    if (speedIncreased(ctx))
    {
        return std::make_unique<StateAccelerating>();
    }
    return nullptr;
}

std::unique_ptr<IDroneState> StateMoving::clone() const
{
    return std::make_unique<StateMoving>();
}

const char* StateMoving::name() const
{
    return "Moving";
}

int StateMoving::code() const
{
    return state_code::MOVING;
}

void executeDroneState(
    std::unique_ptr<IDroneState>& state,
    DroneContext& ctx)
{
    if (state == nullptr)
    {
        state = std::make_unique<StateStopped>();
    }

    auto next = state->execute(ctx);
    if (next)
    {
        state = std::move(next);
    }
    ctx.drone.state = state->code();
}
