#include "types.hpp"

#include <cmath>

Coord Coord::operator+(const Coord& other) const
{
    return {x + other.x, y + other.y};
}

Coord Coord::operator-(const Coord& other) const
{
    return {x - other.x, y - other.y};
}

Coord Coord::operator*(double factor) const
{
    return {x * factor, y * factor};
}

Coord Coord::operator/(double divisor) const
{
    return {x / divisor, y / divisor};
}

bool Coord::operator==(const Coord& other) const
{
    constexpr double epsilon = 1e-9;
    return std::fabs(x - other.x) <= epsilon &&
           std::fabs(y - other.y) <= epsilon;
}
