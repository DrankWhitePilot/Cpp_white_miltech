#pragma once

#include <vector>

#include "types.hpp"

struct TargetSnapshot
{
    double timeSecSinceStart = 0.0;
    std::vector<Target> targets;
};

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
    virtual bool tryPopSnapshot(TargetSnapshot& snapshot) = 0;
};
