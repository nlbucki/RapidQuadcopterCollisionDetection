#pragma once
#include <random>
#include "CommonMath/Vec3.hpp"
#include "RapidQuadcopterTrajectories/SampleGenerator.hpp"

// Generates uniformly distributed position samples in a rectangular prism centered at centerPosition with side lengths sideLen
class TimeSampleGenerator : public SampleGenerator {
 public:
  TimeSampleGenerator(Vec3 position, double minTime, double maxTime)
      : _pos(position),
        _randTime(minTime, maxTime) {
  }
  Vec3 GetPositionSample() {
    return _pos;
  }
  double GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  Vec3 _pos;
  std::default_random_engine _generator;
  std::uniform_real_distribution<double> _randTime, _randX, _randY, _randZ;
};
