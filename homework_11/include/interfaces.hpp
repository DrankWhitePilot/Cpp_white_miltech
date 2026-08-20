#pragma once

#include "types.hpp"

class ITargetProvider {
public:
  virtual int load() = 0;
  virtual int getTargetCount() const = 0;
  virtual int getTimeSteps() const = 0;
  virtual Coord* getTarget(int index) = 0;
  virtual Coord** getTargets() = 0;
  virtual ~ITargetProvider() = default;
};

class IBallisticSolver {
public:
  virtual bool solve(const DroneConfig& config,
                     const AmmoParams& ammo,
                     BallisticResult& result) = 0;

  virtual ~IBallisticSolver() = default;
};

class IConfigLoader {
public:
  virtual int load() = 0;
  virtual const DroneConfig& getConfig() const = 0;
  virtual const AmmoParams& getAmmoParams() const = 0;
  virtual ~IConfigLoader() = default;
};
