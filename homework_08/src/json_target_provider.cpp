#include "json_target_provider.hpp"

#include <fstream>
#include <iostream>
#include <utility>

#include "json.hpp"

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(std::string source)
    : source_(std::move(source))
{
}

int JsonTargetProvider::load()
{
    targets_.clear();
    targetPointers_.clear();
    targetCount_ = 0;
    timeSteps_ = 0;

    std::ifstream fin(source_);
    if (!fin.is_open())
    {
        std::cout << "NO TARGET FILE" << std::endl;
        return 1;
    }

    json j;
    fin >> j;

    targetCount_ = j["targetCount"];

    if (j.contains("timeSteps"))
        timeSteps_ = j["timeSteps"];
    else if (j.contains("targetSteps"))
        timeSteps_ = j["targetSteps"];
    else if (targetCount_ > 0 && j["targets"][0].contains("positions"))
        timeSteps_ = static_cast<int>(j["targets"][0]["positions"].size());
    else if (targetCount_ > 0 && j["targets"][0].contains("points"))
        timeSteps_ = static_cast<int>(j["targets"][0]["points"].size());
    else
    {
        std::cout << "NO TARGET STEPS" << std::endl;
        return 1;
    }

    targets_.reserve(targetCount_);

    for (int i = 0; i < targetCount_; i++)
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

        if (!arr->is_array() || static_cast<int>(arr->size()) < timeSteps_)
        {
            std::cout << "BAD TARGET ARRAY SIZE" << std::endl;
            return 1;
        }

        std::vector<Coord> target;
        target.reserve(timeSteps_);

        for (int k = 0; k < timeSteps_; k++)
        {
            target.push_back({
                (*arr)[k]["x"].get<double>(),
                (*arr)[k]["y"].get<double>()
            });
        }

        targets_.push_back(std::move(target));
    }

    targetPointers_.reserve(targets_.size());

    for (auto& target : targets_)
        targetPointers_.push_back(target.data());

    return 0;
}

int JsonTargetProvider::getTargetCount() const
{
    return targetCount_;
}

int JsonTargetProvider::getTimeSteps() const
{
    return timeSteps_;
}

Coord* JsonTargetProvider::getTarget(int index)
{
    return targets_[index].data();
}

Coord** JsonTargetProvider::getTargets()
{
    return targetPointers_.data();
}
