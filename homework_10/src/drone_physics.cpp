#include "drone_physics.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "model_math.hpp"

DronePhysics::DronePhysics(DroneConfig config)
    : config_(std::move(config)),
      runtime_{config_.startPos,
               config_.initialDir,
               0.0,
               state_code::STOPPED},
      state_(std::make_unique<StateStopped>())
{
    activeCommand_.state = state_code::STOPPED;
    activeCommand_.angleSpeed = config_.angularSpeed;
    activeCommand_.motion = DroneMotion::STOP_AT_POINT;
    activeCommand_.destination = config_.startPos;
    activeCommand_.desiredDirection = config_.initialDir;

    telemetry_.pos = runtime_.position;
    telemetry_.direction = runtime_.direction;
    telemetry_.state = runtime_.state;
}

void DronePhysics::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto startedAt = std::chrono::steady_clock::now();
    const double dt = std::max(config_.physicsTimeStep, 0.001);
    const double scale = std::max(config_.timeScale, 0.001);
    const auto sleepDuration = std::chrono::duration<double>(dt / scale);
    DroneConfig physicsConfig = config_;
    physicsConfig.simTimeStep = dt;

    while (!stopRequested_.load())
    {
        DroneCommand command;
        bool received = false;
        while (commands_.tryPop(command))
        {
            activeCommand_ = command;
            received = true;
        }

        DroneContext context{
            runtime_,
            physicsConfig,
            activeCommand_.motion,
            activeCommand_.destination,
            activeCommand_.desiredDirection};
        executeDroneState(state_, context);

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(
            now - startedAt).count() * scale;
        const Coord velocity = model::directionVector(runtime_.direction) *
                               runtime_.speed;

        {
            std::lock_guard<std::mutex> lock(telemetryMutex_);
            telemetry_ = {
                runtime_.position,
                velocity,
                runtime_.direction,
                runtime_.state,
                elapsed,
                context.completed};
            if (received && !context.completed)
            {
                telemetry_.commandCompleted = false;
            }
        }

        std::this_thread::sleep_for(sleepDuration);
    }
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
}

void DronePhysics::submitCommand(const DroneCommand& command)
{
    commands_.push(command);
}

DroneTelemetry DronePhysics::getTelemetry() const
{
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    return telemetry_;
}
