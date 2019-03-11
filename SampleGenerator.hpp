#pragma once
#include <random>
#include "Common/Math/Vec3.hpp"

template<typename Real>
class SampleGenerator {
 public:
  virtual ~SampleGenerator() {
    // Empty destructor
  }
  virtual Vec3<Real> GetPositionSample() = 0;
  virtual Real GetTimeSample() = 0;
};
