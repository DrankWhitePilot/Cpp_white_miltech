#pragma once

#include "types.hpp"

class ITargetProvider
{
public:
    virtual int load(const char* filename) = 0;
    virtual int getTargetCount() const = 0;
    virtual int getTimeSteps() const = 0;
    virtual Coord* getTarget(int index) = 0;
    virtual Coord** getTargets() = 0;
    virtual ~ITargetProvider() = default;
};

class IBallisticSolver
{
public:
    virtual bool solve(
        InputData& data,
        double droneX,
        double droneY,
        double targetX,
        double targetY,
        double& aimX,
        double& aimY,
        double& fireX,
        double& fireY,
        double& totalTime,
        bool& needManeuver) = 0;

    virtual ~IBallisticSolver() = default;
};

class IConfigLoader
{
public:
    virtual int load(const char* configFile, const char* ammoFile) = 0;
    virtual const DroneConfig& getConfig() const = 0;
    virtual const AmmoParams& getAmmoParams() const = 0;
    virtual ~IConfigLoader() = default;
};
