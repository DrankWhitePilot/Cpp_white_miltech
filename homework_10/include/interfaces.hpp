#pragma once

#include "types.hpp"

class ITargetProvider
{
public:
    virtual ~ITargetProvider() = default;

    virtual int load() = 0;
    virtual void run() = 0;
    virtual bool isThreadReady() const = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual int getTargetCount() const = 0;
    virtual Target getTarget(int index) const = 0;
};

class IBallisticSolver
{
public:
    virtual ~IBallisticSolver() = default;

    virtual bool solve(const DroneConfig& config,
                       const AmmoParams& ammo,
                       const Target& target,
                       int targetIndex,
                       const DroneRuntime& drone,
                       double currentTime,
                       AttackPlan& result) = 0;
};

class IConfigLoader
{
public:
    virtual ~IConfigLoader() = default;

    virtual int load() = 0;
    virtual const DroneConfig& getConfig() const = 0;
    virtual const AmmoParams& getAmmoParams() const = 0;
};
