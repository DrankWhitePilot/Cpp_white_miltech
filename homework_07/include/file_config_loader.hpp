#pragma once

#include <string>

#include "interfaces.hpp"

class FileConfigLoader : public IConfigLoader
{
public:
    FileConfigLoader(const char* configFile, const char* ammoFile);

    int load() override;
    const DroneConfig& getConfig() const override;
    const AmmoParams& getAmmoParams() const override;

private:
    std::string configFile_;
    std::string ammoFile_;
    DroneConfig config_{};
    AmmoParams ammoParams_{};
};
