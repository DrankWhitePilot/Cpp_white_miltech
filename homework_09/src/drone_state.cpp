#include "drone_state.hpp"

namespace
{
template <typename CurrentState>
std::unique_ptr<IDroneState> transitionFrom(DroneContext& ctx, int currentCode)
{
    (void)sizeof(CurrentState);
    if (ctx.requestedState == currentCode)
    {
        ctx.drone.state = currentCode;
        return nullptr;
    }
    if (ctx.requestedState == state_code::STOPPED)
    {
        return std::make_unique<StateStopped>();
    }
    if (ctx.requestedState == state_code::ACCELERATING)
    {
        return std::make_unique<StateAccelerating>();
    }
    if (ctx.requestedState == state_code::DECELERATING)
    {
        return std::make_unique<StateDecelerating>();
    }
    if (ctx.requestedState == state_code::TURNING)
    {
        return std::make_unique<StateTurning>();
    }
    if (ctx.requestedState == state_code::MOVING)
    {
        return std::make_unique<StateMoving>();
    }

    ctx.requestedState = currentCode;
    ctx.drone.state = currentCode;
    return nullptr;
}
}

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx)
{
    return transitionFrom<StateStopped>(ctx, state_code::STOPPED);
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
    return transitionFrom<StateAccelerating>(ctx, state_code::ACCELERATING);
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
    return transitionFrom<StateDecelerating>(ctx, state_code::DECELERATING);
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
    return transitionFrom<StateTurning>(ctx, state_code::TURNING);
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
    return transitionFrom<StateMoving>(ctx, state_code::MOVING);
}

const char* StateMoving::name() const
{
    return "Moving";
}

int StateMoving::code() const
{
    return state_code::MOVING;
}
