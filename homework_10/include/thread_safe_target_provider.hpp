#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "interfaces.hpp"

class ThreadSafeTargetProvider final : public ITargetProvider
{
public:
    ThreadSafeTargetProvider(std::string source,
                             double arrayTimeStep,
                             double targetTimeStep,
                             double timeScale);

    int load() override;
    void run() override;
    bool isThreadReady() const override;
    void start() override;
    void stop() override;
    int getTargetCount() const override;
    Target getTarget(int index) const override;

private:
    void updateTargets();

    std::string source_;
    double arrayTimeStep_;
    double targetTimeStep_;
    double timeScale_;

    std::vector<std::vector<Coord>> trajectories_;
    std::vector<Target> targets_;
    std::vector<std::size_t> indices_;

    mutable std::mutex targetsMutex_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopRequested_{false};
};
