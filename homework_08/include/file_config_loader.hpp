#pragma once

#include <string>

#include "i_config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
  FileConfigLoader(std::string configFile, std::string ammoFile);

  int load() override;
  const DroneConfig& getConfig() const override;
  const AmmoParams& getAmmoParams() const override;

private:
  std::string configFile_;
  std::string ammoFile_;
  DroneConfig config_{};
  AmmoParams ammoParams_{};
};
