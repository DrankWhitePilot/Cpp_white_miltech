#pragma once

#include <atomic>
#include <future>
#include <mutex>

#include "drone_state.hpp"
#include "thread_safe_queue.hpp"
#include "types.hpp"

struct DronePhysicsResult
{
    DroneTelemetry telemetry{};
    bool completed = false;
};

struct DroneCommand
{
    DroneMotion motion = DroneMotion::DYNAMIC;
    Coord destination{0.0, 0.0};
    double desiredDirection = 0.0;
    DroneRuntime runtime{};
    bool completed = false;
    std::shared_ptr<std::promise<DronePhysicsResult>> response;
};

class DronePhysics
{
public:
    DronePhysics() = default;
    explicit DronePhysics(const DroneConfig& config);
    ~DronePhysics();

    DronePhysics(const DronePhysics&) = delete;
    DronePhysics& operator=(const DronePhysics&) = delete;

    void configure(const DroneConfig& config);
    void reset(Coord position, double direction);
    void setDirection(double direction);

    bool isThreadReady() const;
    void start();
    void stop();
    void run();

    DroneTelemetry getTelemetry() const;
    DroneRuntime getRuntime() const;
    DronePhysicsResult executeCommandSync(const DroneCommand& command);

private:
    DronePhysicsResult executeCommandLocked(const DroneCommand& command);

    mutable std::mutex mutex_;
    DroneConfig config_{};
    DroneRuntime runtime_{};
    Coord velocity_{0.0, 0.0};
    double timeSecSinceStart_ = 0.0;
    ThreadSafeQueue<DroneCommand> commands_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
