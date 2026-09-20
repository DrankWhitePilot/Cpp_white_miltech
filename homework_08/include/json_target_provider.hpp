#pragma once

#include <string>
#include <vector>

#include "i_target_provider.hpp"

class JsonTargetProvider : public ITargetProvider
{
public:
    explicit JsonTargetProvider(std::string source);
    ~JsonTargetProvider() override = default;

    int load() override;
    int getTargetCount() const override;
    int getTimeSteps() const override;
    Coord* getTarget(int index) override;
    Coord** getTargets() override;

private:
    std::string source_;
    std::vector<std::vector<Coord>> targets_;
    std::vector<Coord*> targetPointers_;
    int targetCount_ = 0;
    int timeSteps_ = 0;
};
