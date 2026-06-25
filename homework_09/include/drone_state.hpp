#pragma once

#include <memory>

#include "types.hpp"

namespace state_code
{
inline constexpr int STOPPED = 0;
inline constexpr int ACCELERATING = 1;
inline constexpr int DECELERATING = 2;
inline constexpr int TURNING = 3;
inline constexpr int MOVING = 4;
}

struct DroneContext
{
    DroneRuntime& drone;
    const DroneConfig& config;
    int requestedState;
};

class IDroneState
{
public:
    virtual ~IDroneState() = default;
    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
    virtual const char* name() const = 0;
    virtual int code() const = 0;
};

class StateStopped final : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    int code() const override;
};

class StateAccelerating final : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    int code() const override;
};

class StateDecelerating final : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    int code() const override;
};

class StateTurning final : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    int code() const override;
};

class StateMoving final : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    int code() const override;
};
