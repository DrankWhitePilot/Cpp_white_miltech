#include "thread_safe_target_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

    const double scale = std::max(timeScale_, 0.001);
    const double tick = std::max(targetTimeStep_, 0.001);
    const auto period = std::chrono::duration<double>(tick / scale);
    const auto startedAt = std::chrono::steady_clock::now();
    auto nextTick = startedAt;

    while (!stopRequested_.load())
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - startedAt).count() * scale;
        updateTargets(elapsed);

        TargetSnapshot snapshot;
        snapshot.timeSecSinceStart = elapsed;
        {
            std::lock_guard<std::mutex> lock(targetsMutex_);
            snapshot.targets = targets_;
        }
        snapshots_.push(std::move(snapshot));

        nextTick += std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(nextTick);
    }
}

void ThreadSafeTargetProvider::updateTargets(double elapsed)
{
    const double segment = std::max(arrayTimeStep_, model::EPS);
    std::vector<Target> updated;
    updated.reserve(trajectories_.size());

    for (const auto& trajectory : trajectories_)
    {
        if (trajectory.empty())
        {
            updated.push_back({});
            continue;
        }

        const auto absoluteIndex = static_cast<long long>(
            std::floor(elapsed / segment + model::EPS));
        const std::size_t current = static_cast<std::size_t>(
            absoluteIndex % static_cast<long long>(trajectory.size()));
        const std::size_t next = (current + 1U) % trajectory.size();
        const double segmentStart =
            static_cast<double>(absoluteIndex) * segment;
        const double alpha = std::clamp(
            (elapsed - segmentStart) / segment,
            0.0,
            1.0);
        const Coord delta = trajectory[next] - trajectory[current];

        updated.push_back({
            trajectory[current] + delta * alpha,
            delta / segment});
    }

    std::lock_guard<std::mutex> lock(targetsMutex_);
    targets_ = std::move(updated);
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

bool ThreadSafeTargetProvider::tryPopSnapshot(TargetSnapshot& snapshot)
{
    return snapshots_.tryPop(snapshot);
}
