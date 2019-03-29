#pragma once
#include <random>
#include "CommonMath/Vec3.hpp"

class SampleGenerator {
 public:
  virtual ~SampleGenerator() {
    // Empty destructor
  }
  virtual Vec3 GetPositionSample() = 0;
  virtual double GetTimeSample() = 0;
};
