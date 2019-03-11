#pragma once
#include "SampleGenerator.hpp"
#include "Common/Math/Vec3.hpp"
#include <random>

// Generates uniformly distributed position samples in a rectangular prism centered at centerPosition with side lengths sideLen
template<typename Real>
class TimeSampleGenerator : public SampleGenerator<Real> {
 public:
  TimeSampleGenerator(Vec3<Real> position, Real minTime, Real maxTime)
      : _pos(position),
        _randTime(minTime, maxTime) {
  }
  Vec3<Real> GetPositionSample() {
    return _pos;
  }
  Real GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  Vec3<Real> _pos;
  std::default_random_engine _generator;
  std::uniform_real_distribution<Real> _randTime, _randX, _randY, _randZ;
};
