#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "drone_physics.hpp"
#include "drone_state.hpp"
#include "interfaces.hpp"
#include "types.hpp"

class MissionProcessor
{
public:
    MissionProcessor(std::shared_ptr<ITargetProvider> targets,
                     std::shared_ptr<DronePhysics> physics,
                     std::unique_ptr<IBallisticSolver> solver,
                     DroneConfig config,
                     AmmoParams ammo);

    void run();
    bool isThreadReady() const;
    void start();
    void stop();

    const SimStep* getSteps() const;
    int getStepCount() const;

private:
    bool buildPlanForTarget(int targetIndex,
                            const DroneRuntime& drone,
                            double currentTime,
                            AttackPlan& plan) const;
    int chooseBestTarget(const DroneRuntime& drone,
                         double currentTime,
                         AttackPlan& bestPlan) const;
    void initializeMission(const DroneRuntime& drone);
    void processStep(const DroneTelemetry& telemetry);
    void sendCommand(DroneMotion motion,
                     Coord destination,
                     double desiredDirection,
                     int state);
    void appendStep(const DroneTelemetry& telemetry,
                    const AttackPlan& plan);
    void syncStateObject(int stateCode);

    static constexpr int MAX_STEPS = 10000;

    std::shared_ptr<ITargetProvider> targets_;
    std::shared_ptr<DronePhysics> physics_;
    std::unique_ptr<IBallisticSolver> solver_;
    DroneConfig config_;
    AmmoParams ammo_;
    std::unique_ptr<IDroneState> state_;

    MissionRuntime mission_{};
    AttackPlan activePlan_{};
    int currentTargetIndex_ = -1;
    bool initialized_ = false;
    bool finished_ = false;

    std::vector<SimStep> steps_;
    mutable std::mutex stepsMutex_;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
