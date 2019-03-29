#pragma once
#include <random>
#include "RapidQuadcopterTrajectories/SampleGenerator.hpp"
#include "CommonMath/Vec3.hpp"

// Generates uniformly distributed position samples in a rectangular prism centered at centerPosition with side lengths sideLen
class RectPrismSampleGenerator : public SampleGenerator {
 public:
  RectPrismSampleGenerator(Vec3 sideLen, Vec3 centerPosition,
                           double minTime, double maxTime)
      : _randTime(minTime, maxTime),
        _randX(centerPosition.x - sideLen.x / 2,
               centerPosition.x + sideLen.x / 2),
        _randY(centerPosition.y - sideLen.y / 2,
               centerPosition.y + sideLen.y / 2),
        _randZ(centerPosition.z - sideLen.z / 2,
               centerPosition.z + sideLen.z / 2) {
  }
  Vec3 GetPositionSample() {
    return Vec3d(_randX(_generator), _randY(_generator), _randZ(_generator));
  }
  double GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  std::default_random_engine _generator;
  std::uniform_real_distribution _randTime, _randX, _randY, _randZ;
};
