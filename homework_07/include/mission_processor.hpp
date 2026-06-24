#pragma once

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

    int init();
    bool hasNext() const;
    DropPoint step();
    void reset();
    void changeSolver(IBallisticSolver* solver);

private:
    void prepareInputData();
    bool buildPlanForTarget(
        int targetIndex,
        double currentTime,
        AttackPlan& plan) const;

    ITargetProvider* targets_;
    IBallisticSolver* solver_;
    IConfigLoader* loader_;

    DroneRuntime drone_{};
    MissionRuntime mission_{};
    int currentTargetIndex_ = -1;
    bool finished_ = false;
    InputData data_{};
    int currentIdx_ = 0;
    bool initialized_ = false;
};
