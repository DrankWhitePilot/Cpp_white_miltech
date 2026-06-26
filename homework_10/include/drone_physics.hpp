#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "drone_state.hpp"
#include "thread_safe_queue.hpp"
#include "types.hpp"

struct DroneCommand
{
    int state = state_code::STOPPED;
    double angleSpeed = 0.0;
    DroneMotion motion = DroneMotion::DYNAMIC;
    Coord destination{};
    double desiredDirection = 0.0;
};

struct DroneTelemetry
{
    Coord pos{};
    Coord speed{};
    double direction = 0.0;
    int state = state_code::STOPPED;
    double timeSecSinceStart = 0.0;
    bool commandCompleted = false;
};

class DronePhysics
{
public:
    explicit DronePhysics(DroneConfig config);

    void run();
    bool isThreadReady() const;
    void start();
    void stop();

    void submitCommand(const DroneCommand& command);
    DroneTelemetry getTelemetry() const;

private:
    DroneConfig config_;
    DroneRuntime runtime_{};
    std::unique_ptr<IDroneState> state_;
    DroneCommand activeCommand_{};
    ThreadSafeQueue<DroneCommand> commands_;

    mutable std::mutex telemetryMutex_;
    DroneTelemetry telemetry_{};

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
