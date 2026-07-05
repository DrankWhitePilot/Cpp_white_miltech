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
               state_code::STOPPED}
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

    const double nominalDt = std::max(config_.physicsTimeStep, 0.001);
    const double scale = std::max(config_.timeScale, 0.001);
    const auto period = std::chrono::duration<double>(nominalDt / scale);
    const auto startedAt = std::chrono::steady_clock::now();
    auto previousTick = startedAt;
    auto nextTick = startedAt;

    while (!stopRequested_.load())
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsedDt = std::max(
            0.0,
            std::chrono::duration<double>(now - previousTick).count() *
                scale);
        previousTick = now;

        DroneCommand command;
        while (commands_.tryPop(command))
        {
            if (command.id != activeCommand_.id)
            {
                activeCommandCompleted_ = false;
            }
            activeCommand_ = command;
        }

        double remainingDt = elapsedDt;
        while (remainingDt > model::EPS && !activeCommandCompleted_)
        {
            const double stepDt = std::min(remainingDt, nominalDt);
            DroneConfig physicsConfig = config_;
            physicsConfig.simTimeStep = stepDt;
            if (activeCommand_.angleSpeed > model::EPS)
            {
                physicsConfig.angularSpeed = activeCommand_.angleSpeed;
            }

            runtime_.state = activeCommand_.state;
            DroneContext context{
                runtime_,
                physicsConfig,
                activeCommand_.motion,
                activeCommand_.destination,
                activeCommand_.desiredDirection};
            integrateDroneMotion(context);
            activeCommandCompleted_ = context.completed;
            remainingDt -= stepDt;
        }

        runtime_.state = activeCommand_.state;
        const double elapsed =
            std::chrono::duration<double>(now - startedAt).count() * scale;
        const Coord velocity =
            model::directionVector(runtime_.direction) * runtime_.speed;
        {
            std::lock_guard<std::mutex> lock(telemetryMutex_);
            telemetry_ = {
                runtime_.position,
                velocity,
                runtime_.direction,
                runtime_.state,
                elapsed,
                activeCommandCompleted_,
                activeCommandCompleted_ ? activeCommand_.id : 0};
        }

        nextTick += std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(nextTick);
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
