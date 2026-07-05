#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "drone_physics.hpp"
#include "drone_state.hpp"
#include "i_ballistic_solver.hpp"
#include "i_target_provider.hpp"
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
    bool alignTargetsTo(double currentTime);
    void updateTargetMotionHistory(double currentTime);
    int chooseBestTarget(const DroneRuntime& drone,
                         double currentTime,
                         AttackPlan& bestPlan) const;
    void initializeMission(const DroneRuntime& drone,
                           double currentTime);
    void advanceStateFromTelemetry(const DroneTelemetry& telemetry);
    void processStep(const DroneTelemetry& telemetry);
    std::uint64_t sendCommand(DroneMotion motion,
                              Coord destination,
                              double desiredDirection);
    void refreshCommand(std::uint64_t commandId,
                        DroneMotion motion,
                        Coord destination,
                        double desiredDirection);
    void appendStep(const DroneTelemetry& telemetry,
                    const AttackPlan& plan);

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

    std::vector<Target> currentTargets_;
    std::optional<TargetSnapshot> pendingTargetSnapshot_;
    std::vector<std::vector<Coord>> targetPositionHistory_;
    std::vector<Coord> targetSegmentVelocities_;
    std::vector<long long> targetSampleIndices_;

    DroneMotion activeMotion_ = DroneMotion::STOP_AT_POINT;
    Coord activeDestination_{};
    double activeDesiredDirection_ = 0.0;
    double previousDroneSpeed_ = 0.0;
    bool previousTelemetryReady_ = false;

    std::uint64_t nextCommandId_ = 1;
    std::uint64_t activeCommandId_ = 0;
    std::uint64_t phaseCommandId_ = 0;

    bool dropCandidateActive_ = false;
    int dropCandidateTargetIndex_ = -1;
    double dropCandidateError_ = 0.0;
    int dropCandidateImprovements_ = 0;

    bool initialized_ = false;
    bool finished_ = false;

    std::vector<SimStep> steps_;
    mutable std::mutex stepsMutex_;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
