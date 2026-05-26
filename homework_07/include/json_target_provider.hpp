#pragma once

#include "interfaces.hpp"

class JsonTargetProvider : public ITargetProvider
{
public:
    JsonTargetProvider() = default;
    ~JsonTargetProvider() override;

    int load(const char* filename) override;
    int getTargetCount() const override;
    int getTimeSteps() const override;
    Coord* getTarget(int index) override;
    Coord** getTargets() override;

private:
    void clear();

    Coord** targets_ = nullptr;
    int targetCount_ = 0;
    int timeSteps_ = 0;
};
