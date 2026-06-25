#pragma once

#include <memory>
#include <vector>

#include "drone_state.hpp"
#include "interfaces.hpp"
#include "types.hpp"

class MissionProcessor
{
public:
    MissionProcessor(
        std::unique_ptr<ITargetProvider> targets,
        std::unique_ptr<IBallisticSolver> solver,
        std::unique_ptr<IConfigLoader> loader);
    ~MissionProcessor();

    MissionProcessor(const MissionProcessor&) = delete;
    MissionProcessor& operator=(const MissionProcessor&) = delete;
    MissionProcessor(MissionProcessor&&) noexcept = default;
    MissionProcessor& operator=(MissionProcessor&&) noexcept = default;

    int init();
    bool hasNext() const;
    void step();
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> solver);

    const SimStep* getSteps() const;
    int getStepCount() const;

private:
    bool buildPlanForTarget(
        int targetIndex,
        double currentTime,
        AttackPlan& plan) const;

    int chooseBestTargetFromState(
        double currentTime,
        int currentTargetIndex,
        AttackPlan& bestPlan) const;

    bool appendStep(const SimStep& step);
    void initializeRuntime();
    void synchronizeState(int requestedState);

    static constexpr int MAX_STEPS = 10000;

    std::unique_ptr<ITargetProvider> targets_;
    std::unique_ptr<IBallisticSolver> solver_;
    std::unique_ptr<IConfigLoader> loader_;
    std::unique_ptr<IDroneState> state_;

    DroneRuntime drone_{};
    MissionRuntime mission_{};
    int currentTargetIndex_ = -1;

    std::vector<SimStep> steps_;
    int stepCount_ = 0;

    bool initialized_ = false;
    bool finished_ = true;
};
