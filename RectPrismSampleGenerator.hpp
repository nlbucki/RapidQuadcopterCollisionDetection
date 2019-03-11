#pragma once
#include "SampleGenerator.hpp"
#include "Common/Math/Vec3.hpp"
#include <random>

// Generates uniformly distributed position samples in a rectangular prism centered at centerPosition with side lengths sideLen
template<typename Real>
class RectPrismSampleGenerator : public SampleGenerator<Real> {
 public:
  RectPrismSampleGenerator(Vec3<Real> sideLen, Vec3<Real> centerPosition,
                           Real minTime, Real maxTime)
      : _randTime(minTime, maxTime),
        _randX(centerPosition.x - sideLen.x / 2,
               centerPosition.x + sideLen.x / 2),
        _randY(centerPosition.y - sideLen.y / 2,
               centerPosition.y + sideLen.y / 2),
        _randZ(centerPosition.z - sideLen.z / 2,
               centerPosition.z + sideLen.z / 2) {
  }
  Vec3<Real> GetPositionSample() {
    return Vec3d(_randX(_generator), _randY(_generator), _randZ(_generator));
  }
  Real GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  std::default_random_engine _generator;
  std::uniform_real_distribution<Real> _randTime, _randX, _randY, _randZ;
};
