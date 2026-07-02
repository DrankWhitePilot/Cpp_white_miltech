#include "file_config_loader.hpp"

#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include "json.hpp"

using json = nlohmann::json;

namespace
{
std::vector<AmmoParams> loadAmmo(const std::string& filename, bool& loaded)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cout << "NO AMMO FILE" << std::endl;
        loaded = false;
        return {};
    }

    json j;
    fin >> j;

    std::vector<AmmoParams> ammo;
    ammo.reserve(j.size());

    for (const auto& item : j)
    {
        ammo.push_back({
            item["name"].get<std::string>(),
            item["mass"].get<double>(),
            item["drag"].get<double>(),
            item["lift"].get<double>()
        });
    }

    loaded = true;
    return ammo;
}
}

FileConfigLoader::FileConfigLoader(std::string configFile, std::string ammoFile)
    : configFile_(std::move(configFile)),
      ammoFile_(std::move(ammoFile))
{
}

int FileConfigLoader::load()
{
    std::ifstream fin(configFile_);
    if (!fin.is_open())
    {
        std::cout << "NO FILE" << std::endl;
        return 1;
    }

    json j;
    fin >> j;

    config_.startPos.x = j["drone"]["position"]["x"];
    config_.startPos.y = j["drone"]["position"]["y"];
    config_.altitude = j["drone"]["altitude"];
    config_.initialDir = j["drone"]["initialDirection"];
    config_.attackSpeed = j["drone"]["attackSpeed"];
    config_.accelPath = j["drone"]["accelerationPath"];
    config_.angularSpeed = j["drone"]["angularSpeed"];
    config_.turnThreshold = j["drone"]["turnThreshold"];
    config_.ammoName = j["ammo"].get<std::string>();

    config_.simTimeStep = j["simulation"]["timeStep"];
    config_.hitRadius = j["simulation"]["hitRadius"];
    config_.arrayTimeStep = j["targetArrayTimeStep"];

    bool ammoLoaded = false;
    const std::vector<AmmoParams> ammo = loadAmmo(ammoFile_, ammoLoaded);

    if (!ammoLoaded)
        return 1;

    for (const auto& item : ammo)
    {
        if (config_.ammoName == item.name)
        {
            ammoParams_ = item;
            return 0;
        }
    }

    std::cout << "Unknown ammo type" << std::endl;
    return 1;
}

const DroneConfig& FileConfigLoader::getConfig() const
{
    return config_;
}

const AmmoParams& FileConfigLoader::getAmmoParams() const
{
    return ammoParams_;
}
