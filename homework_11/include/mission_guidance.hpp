#pragma once

#include "types.hpp"

struct MissionDecision
{
    bool valid = false;
    Coord destination{0.0, 0.0};
    double desiredSpeed = 0.0;
    bool drop = false;
    int targetIndex = -1;
    Coord aimPoint{0.0, 0.0};
    Coord predictedTarget{0.0, 0.0};
};
