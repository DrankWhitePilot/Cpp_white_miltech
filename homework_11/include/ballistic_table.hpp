#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct BallisticTable
{
    struct Result
    {
        double t = 0.0;
        double hDist = 0.0;
    };

    std::vector<double> axisZ0;
    std::vector<double> axisV0;
    std::vector<double> axisM;
    std::vector<double> axisD;
    std::vector<double> axisL;
    std::vector<Result> data;

    bool load(const std::string& path);
    bool valid() const;
    Result lookup(double z0, double v0, double m, double d, double l) const;

    std::size_t index(int iz, int iv, int im, int id, int il) const;
    const Result& at(int iz, int iv, int im, int id, int il) const;
};
