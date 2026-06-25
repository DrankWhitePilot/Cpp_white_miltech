#include "drone_state.hpp"

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
}

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx)
{
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

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx)
{
    if (ctx.turnRequired || speedDecreased(ctx))
    {
        return std::make_unique<StateDecelerating>();
    }
    if (isStopped(ctx))
    {
        return std::make_unique<StateStopped>();
    }
    if (!speedIncreased(ctx) &&
        ctx.drone.speed >= ctx.config.attackSpeed - model::EPS)
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

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx)
{
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
