#include "thread_safe_target_provider.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <utility>

#include "json.hpp"
#include "model_math.hpp"

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(
    std::string source,
    double arrayTimeStep,
    double targetTimeStep,
    double timeScale)
    : source_(std::move(source)),
      arrayTimeStep_(arrayTimeStep),
      targetTimeStep_(targetTimeStep),
      timeScale_(timeScale)
{
}

int ThreadSafeTargetProvider::load()
{
    std::ifstream input(source_);
    if (!input.is_open())
    {
        std::cerr << "NO TARGET FILE\n";
        return 1;
    }

    json data;
    input >> data;

    const int targetCount = data.value("targetCount", 0);
    if (targetCount <= 0 || !data.contains("targets"))
    {
        return 1;
    }

    std::vector<std::vector<Coord>> loaded;
    loaded.reserve(static_cast<std::size_t>(targetCount));

    for (int i = 0; i < targetCount; ++i)
    {
        const json& item = data["targets"][i];
        const json* points = nullptr;
        if (item.contains("positions"))
        {
            points = &item["positions"];
        }
        else if (item.contains("points"))
        {
            points = &item["points"];
        }

        if (points == nullptr || !points->is_array() || points->empty())
        {
            return 1;
        }

        std::vector<Coord> trajectory;
        trajectory.reserve(points->size());
        for (const auto& point : *points)
        {
            trajectory.push_back({point["x"].get<double>(),
                                  point["y"].get<double>()});
        }
        loaded.push_back(std::move(trajectory));
    }

    std::vector<Target> snapshots;
    snapshots.reserve(loaded.size());
    for (const auto& trajectory : loaded)
    {
        Coord velocity{0.0, 0.0};
        if (trajectory.size() > 1 && arrayTimeStep_ > model::EPS)
        {
            velocity = (trajectory[1] - trajectory[0]) / arrayTimeStep_;
        }
        snapshots.push_back({trajectory.front(), velocity});
    }

    {
        std::lock_guard<std::mutex> lock(targetsMutex_);
        trajectories_ = std::move(loaded);
        targets_ = std::move(snapshots);
        indices_.assign(targets_.size(), 0U);
    }
    return 0;
}

void ThreadSafeTargetProvider::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const double safeScale = std::max(timeScale_, 0.001);
    const double safeStep = std::max(targetTimeStep_, 0.001);
    const auto sleepDuration = std::chrono::duration<double>(safeStep / safeScale);

    while (!stopRequested_.load())
    {
        std::this_thread::sleep_for(sleepDuration);
        if (!stopRequested_.load())
        {
            updateTargets();
        }
    }
}

void ThreadSafeTargetProvider::updateTargets()
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    for (std::size_t i = 0; i < trajectories_.size(); ++i)
    {
        const auto& trajectory = trajectories_[i];
        if (trajectory.empty())
        {
            continue;
        }

        const std::size_t previous = indices_[i];
        const std::size_t next = (previous + 1U) % trajectory.size();
        const double dt = std::max(arrayTimeStep_, model::EPS);
        targets_[i].pos = trajectory[next];
        targets_[i].velocity = (trajectory[next] - trajectory[previous]) / dt;
        indices_[i] = next;
    }
}

bool ThreadSafeTargetProvider::isThreadReady() const
{
    return ready_.load();
}

void ThreadSafeTargetProvider::start()
{
    started_.store(true);
}

void ThreadSafeTargetProvider::stop()
{
    stopRequested_.store(true);
}

int ThreadSafeTargetProvider::getTargetCount() const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    return static_cast<int>(targets_.size());
}

Target ThreadSafeTargetProvider::getTarget(int index) const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    if (index < 0 || index >= static_cast<int>(targets_.size()))
    {
        return {};
    }
    return targets_[static_cast<std::size_t>(index)];
}
