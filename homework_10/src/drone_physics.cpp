#include "drone_physics.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "model_math.hpp"

namespace
{
Coord velocityFromRuntime(const DroneRuntime& drone)
{
    return model::directionVector(drone.direction) * drone.speed;
}
}

DronePhysics::DronePhysics(const DroneConfig& config)
{
    configure(config);
}

DronePhysics::~DronePhysics()
{
    stop();
}

void DronePhysics::configure(const DroneConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

void DronePhysics::reset(Coord position, double direction)
{
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_ = {position, direction, 0.0, state_code::STOPPED};
    velocity_ = {0.0, 0.0};
    timeSecSinceStart_ = 0.0;
}

void DronePhysics::setDirection(double direction)
{
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_.direction = direction;
    velocity_ = velocityFromRuntime(runtime_);
}

bool DronePhysics::isThreadReady() const
{
    return ready_.load();
}

void DronePhysics::start()
{
    started_.store(true);
}

void DronePhysics::stop()
{
    stopRequested_.store(true);
    commands_.close();
}

DroneTelemetry DronePhysics::getTelemetry() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {
        runtime_.position,
        velocity_,
        runtime_.direction,
        runtime_.state,
        timeSecSinceStart_};
}

DroneRuntime DronePhysics::getRuntime() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_;
}

DronePhysicsResult DronePhysics::executeCommandLocked(const DroneCommand& command)
{
    runtime_ = command.runtime;
    velocity_ = velocityFromRuntime(runtime_);
    timeSecSinceStart_ += config_.simTimeStep;

    return {
        {runtime_.position,
         velocity_,
         runtime_.direction,
         runtime_.state,
         timeSecSinceStart_},
        command.completed};
}

DronePhysicsResult DronePhysics::executeCommandSync(const DroneCommand& command)
{
    if (!ready_.load() || !started_.load() || stopRequested_.load())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return executeCommandLocked(command);
    }

    DroneCommand queued = command;
    queued.response = std::make_shared<std::promise<DronePhysicsResult>>();
    auto future = queued.response->get_future();
    commands_.push(std::move(queued));
    return future.get();
}

void DronePhysics::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (!stopRequested_.load())
    {
        DroneCommand command;
        if (!commands_.waitPop(command))
        {
            break;
        }

        DronePhysicsResult result;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result = executeCommandLocked(command);
        }

        if (command.response)
        {
            command.response->set_value(result);
        }
    }
}
