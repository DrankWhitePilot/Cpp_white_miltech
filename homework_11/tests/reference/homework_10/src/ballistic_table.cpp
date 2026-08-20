#include "ballistic_table.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace
{
struct Interp
{
    int lo = 0;
    double frac = 0.0;
};

BallisticTable::Result lerp(
    const BallisticTable::Result& a,
    const BallisticTable::Result& b,
    double fraction)
{
    return {
        a.t + (b.t - a.t) * fraction,
        a.hDist + (b.hDist - a.hDist) * fraction};
}

Interp findInterp(double value, const std::vector<double>& axis)
{
    if (axis.size() < 2)
    {
        throw std::runtime_error("Ballistic table axis must contain at least two values");
    }
    if (value <= axis.front())
    {
        return {0, 0.0};
    }
    if (value >= axis.back())
    {
        return {static_cast<int>(axis.size()) - 2, 1.0};
    }

    auto upper = std::lower_bound(axis.begin(), axis.end(), value);
    int lo = static_cast<int>(upper - axis.begin()) - 1;
    double denominator = axis[lo + 1] - axis[lo];
    if (denominator <= 0.0)
    {
        throw std::runtime_error("Ballistic table axis must be strictly increasing");
    }
    return {lo, (value - axis[lo]) / denominator};
}

bool strictlyIncreasing(const std::vector<double>& axis)
{
    return axis.size() >= 2 &&
           std::adjacent_find(
               axis.begin(),
               axis.end(),
               [](double a, double b) { return b <= a; }) == axis.end();
}
}

bool BallisticTable::load(const std::string& path)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        return false;
    }

    int nZ = 0;
    int nV = 0;
    int nM = 0;
    int nD = 0;
    int nL = 0;
    if (!(input >> nZ >> nV >> nM >> nD >> nL) ||
        nZ < 2 || nV < 2 || nM < 2 || nD < 2 || nL < 2)
    {
        return false;
    }

    axisZ0.resize(static_cast<std::size_t>(nZ));
    axisV0.resize(static_cast<std::size_t>(nV));
    axisM.resize(static_cast<std::size_t>(nM));
    axisD.resize(static_cast<std::size_t>(nD));
    axisL.resize(static_cast<std::size_t>(nL));

    for (double& value : axisZ0) input >> value;
    for (double& value : axisV0) input >> value;
    for (double& value : axisM) input >> value;
    for (double& value : axisD) input >> value;
    for (double& value : axisL) input >> value;

    if (!input ||
        !strictlyIncreasing(axisZ0) ||
        !strictlyIncreasing(axisV0) ||
        !strictlyIncreasing(axisM) ||
        !strictlyIncreasing(axisD) ||
        !strictlyIncreasing(axisL))
    {
        return false;
    }

    std::size_t total = static_cast<std::size_t>(nZ) *
                        static_cast<std::size_t>(nV) *
                        static_cast<std::size_t>(nM) *
                        static_cast<std::size_t>(nD) *
                        static_cast<std::size_t>(nL);
    data.resize(total);
    for (Result& result : data)
    {
        if (!(input >> result.t >> result.hDist))
        {
            data.clear();
            return false;
        }
    }
    return valid();
}

bool BallisticTable::valid() const
{
    if (!strictlyIncreasing(axisZ0) ||
        !strictlyIncreasing(axisV0) ||
        !strictlyIncreasing(axisM) ||
        !strictlyIncreasing(axisD) ||
        !strictlyIncreasing(axisL))
    {
        return false;
    }

    std::size_t expected = axisZ0.size() * axisV0.size() * axisM.size() *
                           axisD.size() * axisL.size();
    return data.size() == expected;
}

std::size_t BallisticTable::index(
    int iz,
    int iv,
    int im,
    int id,
    int il) const
{
    return ((((static_cast<std::size_t>(iz) * axisV0.size() +
               static_cast<std::size_t>(iv)) *
                  axisM.size() +
              static_cast<std::size_t>(im)) *
                 axisD.size() +
             static_cast<std::size_t>(id)) *
                axisL.size() +
            static_cast<std::size_t>(il));
}

const BallisticTable::Result& BallisticTable::at(
    int iz,
    int iv,
    int im,
    int id,
    int il) const
{
    return data.at(index(iz, iv, im, id, il));
}

BallisticTable::Result BallisticTable::lookup(
    double z0,
    double v0,
    double m,
    double d,
    double l) const
{
    if (!valid())
    {
        throw std::runtime_error("Ballistic table is not loaded");
    }

    Interp iz = findInterp(z0, axisZ0);
    Interp iv = findInterp(v0, axisV0);
    Interp im = findInterp(m, axisM);
    Interp id = findInterp(d, axisD);
    Interp il = findInterp(l, axisL);

    Result alongL[16];
    for (int a = 0; a < 2; ++a)
    {
        for (int b = 0; b < 2; ++b)
        {
            for (int c = 0; c < 2; ++c)
            {
                for (int e = 0; e < 2; ++e)
                {
                    const Result& low = at(
                        iz.lo + a,
                        iv.lo + b,
                        im.lo + c,
                        id.lo + e,
                        il.lo);
                    const Result& high = at(
                        iz.lo + a,
                        iv.lo + b,
                        im.lo + c,
                        id.lo + e,
                        il.lo + 1);
                    alongL[a * 8 + b * 4 + c * 2 + e] =
                        lerp(low, high, il.frac);
                }
            }
        }
    }

    Result alongD[8];
    for (int a = 0; a < 2; ++a)
    {
        for (int b = 0; b < 2; ++b)
        {
            for (int c = 0; c < 2; ++c)
            {
                alongD[a * 4 + b * 2 + c] = lerp(
                    alongL[a * 8 + b * 4 + c * 2],
                    alongL[a * 8 + b * 4 + c * 2 + 1],
                    id.frac);
            }
        }
    }

    Result alongM[4];
    for (int a = 0; a < 2; ++a)
    {
        for (int b = 0; b < 2; ++b)
        {
            alongM[a * 2 + b] = lerp(
                alongD[a * 4 + b * 2],
                alongD[a * 4 + b * 2 + 1],
                im.frac);
        }
    }

    Result alongV[2];
    for (int a = 0; a < 2; ++a)
    {
        alongV[a] = lerp(
            alongM[a * 2],
            alongM[a * 2 + 1],
            iv.frac);
    }

    return lerp(alongV[0], alongV[1], iz.frac);
}
