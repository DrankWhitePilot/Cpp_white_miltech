#pragma once

#include <vector>

class ITargetProvider;
class IBallisticSolver;
class IConfigLoader;

#include "types.hpp"

class MissionProcessor
{
public:
    MissionProcessor(
        ITargetProvider* targets,
        IBallisticSolver* solver,
        IConfigLoader* loader);
    ~MissionProcessor() = default;

    int init();
    bool hasNext() const;
    void step();
    void reset();
    void changeSolver(IBallisticSolver* solver);

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

    static constexpr int MAX_STEPS = 10000;

    ITargetProvider* targets_;
    IBallisticSolver* solver_;
    IConfigLoader* loader_;

    DroneRuntime drone_{};
    MissionRuntime mission_{};
    int currentTargetIndex_ = -1;

    std::vector<SimStep> steps_;
    int stepCount_ = 0;

    bool initialized_ = false;
    bool finished_ = true;
};
