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
