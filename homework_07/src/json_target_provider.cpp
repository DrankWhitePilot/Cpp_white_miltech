#include "json_target_provider.hpp"

#include <fstream>
#include <iostream>

#include "json.hpp"

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const char* source)
    : source_(source != nullptr ? source : "")
{
}

JsonTargetProvider::~JsonTargetProvider()
{
    clear();
}

void JsonTargetProvider::clear()
{
    if (targets_ != nullptr)
    {
        for (int i = 0; i < targetCount_; i++)
        {
            delete[] targets_[i];
        }

        delete[] targets_;
    }

    targets_ = nullptr;
    targetCount_ = 0;
    timeSteps_ = 0;
}

int JsonTargetProvider::load()
{
    clear();

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
        timeSteps_ = (int)j["targets"][0]["positions"].size();
    else if (targetCount_ > 0 && j["targets"][0].contains("points"))
        timeSteps_ = (int)j["targets"][0]["points"].size();
    else
    {
        std::cout << "NO TARGET STEPS" << std::endl;
        return 1;
    }

    targets_ = new Coord*[targetCount_];

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

        if (!arr->is_array() || (int)arr->size() < timeSteps_)
        {
            std::cout << "BAD TARGET ARRAY SIZE" << std::endl;
            return 1;
        }

        targets_[i] = new Coord[timeSteps_];

        for (int k = 0; k < timeSteps_; k++)
        {
            targets_[i][k].x = (*arr)[k]["x"];
            targets_[i][k].y = (*arr)[k]["y"];
        }
    }

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
    return targets_[index];
}

Coord** JsonTargetProvider::getTargets()
{
    return targets_;
}
