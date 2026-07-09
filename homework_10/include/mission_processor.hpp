#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "drone_physics.hpp"
#include "interfaces.hpp"
#include "types.hpp"

class MissionProcessor
{
public:
    MissionProcessor(
        std::unique_ptr<ITargetProvider> targets,
        std::unique_ptr<IBallisticSolver> solver,
        std::unique_ptr<IConfigLoader> loader,
        std::shared_ptr<DronePhysics> physics);
    ~MissionProcessor();

    MissionProcessor(const MissionProcessor&) = delete;
    MissionProcessor& operator=(const MissionProcessor&) = delete;
    MissionProcessor(MissionProcessor&&) noexcept = delete;
    MissionProcessor& operator=(MissionProcessor&&) noexcept = delete;

    int init();
    bool hasNext() const;
    void step();
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> solver);

    void run();
    bool isThreadReady() const;
    void start();
    void stop();

    const SimStep* getSteps() const;
    int getStepCount() const;

private:
    bool buildPlanForTarget(
        int targetIndex,
        double currentTime,
        const DroneRuntime& drone,
        AttackPlan& plan) const;

    int chooseBestTargetFromState(
        double currentTime,
        int currentTargetIndex,
        const DroneRuntime& drone,
        AttackPlan& bestPlan) const;

    bool appendStep(const SimStep& step);
    void initializeRuntime();

    static constexpr int MAX_STEPS = 10000;

    std::unique_ptr<ITargetProvider> targets_;
    std::unique_ptr<IBallisticSolver> solver_;
    std::unique_ptr<IConfigLoader> loader_;
    std::shared_ptr<DronePhysics> physics_;
    std::unique_ptr<IDroneState> state_ = std::make_unique<StateStopped>();

    MissionRuntime mission_{};
    int currentTargetIndex_ = -1;

    std::vector<SimStep> steps_;
    int stepCount_ = 0;

    bool initialized_ = false;
    bool finished_ = true;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
    mutable std::mutex stepsMutex_;
};
