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

ThreadSafeTargetProvider::ThreadSafeTargetProvider(std::string source)
    : source_(std::move(source))
{
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider()
{
    stop();
}

int ThreadSafeTargetProvider::load()
{
    std::ifstream fin(source_);
    if (!fin.is_open())
    {
        std::cout << "NO TARGET FILE" << std::endl;
        return 1;
    }

    json j;
    fin >> j;

    std::vector<std::vector<Coord>> loadedPaths;
    std::vector<Coord*> loadedPointers;
    std::vector<Target> loadedCurrent;

    int loadedTargetCount = j["targetCount"];
    int loadedTimeSteps = 0;

    if (j.contains("timeSteps"))
        loadedTimeSteps = j["timeSteps"];
    else if (j.contains("targetSteps"))
        loadedTimeSteps = j["targetSteps"];
    else if (loadedTargetCount > 0 && j["targets"][0].contains("positions"))
        loadedTimeSteps = static_cast<int>(j["targets"][0]["positions"].size());
    else if (loadedTargetCount > 0 && j["targets"][0].contains("points"))
        loadedTimeSteps = static_cast<int>(j["targets"][0]["points"].size());
    else
    {
        std::cout << "NO TARGET STEPS" << std::endl;
        return 1;
    }

    loadedPaths.reserve(loadedTargetCount);
    for (int i = 0; i < loadedTargetCount; ++i)
    {
        const json* arr = nullptr;
        if (j["targets"][i].contains("positions"))
            arr = &j["targets"][i]["positions"];
        else if (j["targets"][i].contains("points"))
            arr = &j["targets"][i]["points"];
        else
        {
            std::cout << "NO TARGET POINT ARRAY" << std::endl;
            return 1;
        }

        if (!arr->is_array() || static_cast<int>(arr->size()) < loadedTimeSteps)
        {
            std::cout << "BAD TARGET ARRAY SIZE" << std::endl;
            return 1;
        }

        std::vector<Coord> target;
        target.reserve(loadedTimeSteps);
        for (int k = 0; k < loadedTimeSteps; ++k)
        {
            target.push_back({
                (*arr)[k]["x"].get<double>(),
                (*arr)[k]["y"].get<double>()});
        }
        loadedPaths.push_back(std::move(target));
    }

    loadedPointers.reserve(loadedPaths.size());
    for (auto& path : loadedPaths)
    {
        loadedPointers.push_back(path.data());
    }

    loadedCurrent.resize(static_cast<std::size_t>(loadedTargetCount));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        paths_ = std::move(loadedPaths);
        targetPointers_.clear();
        targetPointers_.reserve(paths_.size());
        for (auto& path : paths_)
        {
            targetPointers_.push_back(path.data());
        }
        currentTargets_ = std::move(loadedCurrent);
        targetCount_ = loadedTargetCount;
        timeSteps_ = loadedTimeSteps;
        updateCurrentTargetsLocked(0);
    }

    return 0;
}

int ThreadSafeTargetProvider::getTargetCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return targetCount_;
}

int ThreadSafeTargetProvider::getTimeSteps() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timeSteps_;
}

Coord* ThreadSafeTargetProvider::getTarget(int index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return paths_[static_cast<std::size_t>(index)].data();
}

Coord** ThreadSafeTargetProvider::getTargets()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return targetPointers_.data();
}

void ThreadSafeTargetProvider::setTiming(double arrayTimeStep, double timeScale)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (arrayTimeStep > model::EPS)
    {
        arrayTimeStep_ = arrayTimeStep;
    }
    if (timeScale > model::EPS)
    {
        timeScale_ = timeScale;
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

Target ThreadSafeTargetProvider::getCurrentTarget(int index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return currentTargets_[static_cast<std::size_t>(index)];
}

std::vector<Target> ThreadSafeTargetProvider::getSnapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return currentTargets_;
}

Coord ThreadSafeTargetProvider::calcVelocity(int targetIndex, int sampleIndex) const
{
    if (timeSteps_ <= 1 || arrayTimeStep_ <= model::EPS || sampleIndex <= 0)
    {
        return {0.0, 0.0};
    }

    const int previousIndex = sampleIndex - 1;
    const Coord current = paths_[static_cast<std::size_t>(targetIndex)]
                               [static_cast<std::size_t>(sampleIndex)];
    const Coord previous = paths_[static_cast<std::size_t>(targetIndex)]
                                [static_cast<std::size_t>(previousIndex)];
    return (current - previous) / arrayTimeStep_;
}

void ThreadSafeTargetProvider::updateCurrentTargetsLocked(int sampleIndex)
{
    if (targetCount_ <= 0 || timeSteps_ <= 0)
    {
        return;
    }

    const int wrappedIndex = sampleIndex % timeSteps_;
    for (int i = 0; i < targetCount_; ++i)
    {
        currentTargets_[static_cast<std::size_t>(i)] = {
            paths_[static_cast<std::size_t>(i)][static_cast<std::size_t>(wrappedIndex)],
            calcVelocity(i, wrappedIndex)};
    }
}

void ThreadSafeTargetProvider::run()
{
    ready_.store(true);
    while (!stopRequested_.load() && !started_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    int sampleIndex = 0;
    while (!stopRequested_.load())
    {
        double step = 0.1;
        double scale = 1000.0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            updateCurrentTargetsLocked(sampleIndex);
            step = arrayTimeStep_;
            scale = timeScale_;
        }
        ++sampleIndex;
        const double sleepSeconds = step / std::max(scale, model::EPS);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(sleepSeconds));
    }
}
