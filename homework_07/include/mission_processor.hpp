#pragma once

#include "interfaces.hpp"
#include "types.hpp"

class MissionProcessor
{
public:
    MissionProcessor(
        ITargetProvider* targets,
        IBallisticSolver* solver,
        IConfigLoader* loader);

    int init(const char* configFile, const char* ammoFile);
    bool hasNext() const;
    DropPoint step();
    void reset();
    void changeSolver(IBallisticSolver* solver);

private:
    void prepareInputData();

    ITargetProvider* targets_;
    IBallisticSolver* solver_;
    IConfigLoader* loader_;
    InputData data_{};
    int currentIdx_ = 0;
    bool initialized_ = false;
};
