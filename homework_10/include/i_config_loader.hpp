#pragma once

#include "types.hpp"

class IConfigLoader
{
public:
    virtual ~IConfigLoader() = default;

    virtual int load() = 0;
    virtual const DroneConfig& getConfig() const = 0;
    virtual const AmmoParams& getAmmoParams() const = 0;
};
