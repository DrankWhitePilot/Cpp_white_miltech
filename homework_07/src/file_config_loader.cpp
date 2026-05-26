#include "file_config_loader.hpp"

#include <cstring>
#include <fstream>
#include <iostream>

#include "json.hpp"

using json = nlohmann::json;

namespace
{
void copyJsonText(char* dest, int maxSize, const json& value)
{
    strncpy(dest, value.dump().c_str() + 1, maxSize - 1);
    dest[maxSize - 1] = '\0';

    int len = strlen(dest);
    if (len > 0 && dest[len - 1] == '"')
        dest[len - 1] = '\0';
}

int loadAmmo(AmmoParams*& ammo, int& count, const char* filename)
{
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cout << "NO AMMO FILE" << std::endl;
        return 1;
    }

    json j;
    fin >> j;

    count = j.size();
    ammo = new AmmoParams[count];

    for (int i = 0; i < count; i++)
    {
        copyJsonText(ammo[i].name, 32, j[i]["name"]);
        ammo[i].mass = j[i]["mass"];
        ammo[i].drag = j[i]["drag"];
        ammo[i].lift = j[i]["lift"];
    }

    return 0;
}
}

int FileConfigLoader::load(const char* configFile, const char* ammoFile)
{
    std::ifstream fin(configFile);
    if (!fin.is_open()) {
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

    copyJsonText(config_.ammoName, 32, j["ammo"]);

    config_.simTimeStep = j["simulation"]["timeStep"];
    config_.hitRadius = j["simulation"]["hitRadius"];
    config_.arrayTimeStep = j["targetArrayTimeStep"];

    AmmoParams* ammo = nullptr;
    int ammoCount = 0;

    if (loadAmmo(ammo, ammoCount, ammoFile) != 0)
    {
        delete[] ammo;
        return 1;
    }

    for (int i = 0; i < ammoCount; i++)
    {
        if (strcmp(config_.ammoName, ammo[i].name) == 0)
        {
            ammoParams_ = ammo[i];
            delete[] ammo;
            return 0;
        }
    }

    std::cout << "Unknown ammo type" << std::endl;
    delete[] ammo;
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
