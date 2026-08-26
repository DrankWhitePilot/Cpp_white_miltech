#pragma once

#include <string>
#include "interfaces.hpp"

class JsonTargetProvider : public ITargetProvider
{
public:
    explicit JsonTargetProvider(const char* source);
    ~JsonTargetProvider() override;

    int load() override;
    int getTargetCount() const override;
    int getTimeSteps() const override;
    Coord* getTarget(int index) override;
    Coord** getTargets() override;

private:
    std::string source_;
    void clear();

    Coord** targets_ = nullptr;
    int targetCount_ = 0;
    int timeSteps_ = 0;
};
