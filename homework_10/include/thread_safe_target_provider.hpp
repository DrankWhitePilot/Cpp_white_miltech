#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "interfaces.hpp"

class ThreadSafeTargetProvider final : public ITargetProvider
{
public:
    explicit ThreadSafeTargetProvider(std::string source);
    ~ThreadSafeTargetProvider() override;

    int load() override;
    int getTargetCount() const override;
    int getTimeSteps() const override;
    Coord* getTarget(int index) override;
    Coord** getTargets() override;

    void setTiming(double arrayTimeStep, double timeScale);
    bool isThreadReady() const;
    void start();
    void stop();
    void run();

    Target getCurrentTarget(int index) const;
    std::vector<Target> getSnapshot() const;

private:
    void updateCurrentTargetsLocked(int sampleIndex);
    Coord calcVelocity(int targetIndex, int sampleIndex) const;

    std::string source_;
    std::vector<std::vector<Coord>> paths_;
    std::vector<Coord*> targetPointers_;
    std::vector<Target> currentTargets_;

    int targetCount_ = 0;
    int timeSteps_ = 0;
    double arrayTimeStep_ = 0.1;
    double timeScale_ = 1000.0;

    mutable std::mutex mutex_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
