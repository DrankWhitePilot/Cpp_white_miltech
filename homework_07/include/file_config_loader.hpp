#pragma once

#include "interfaces.hpp"

class FileConfigLoader : public IConfigLoader
{
public:
    int load(const char* configFile, const char* ammoFile) override;
    const DroneConfig& getConfig() const override;
    const AmmoParams& getAmmoParams() const override;

private:
    DroneConfig config_{};
    AmmoParams ammoParams_{};
};
