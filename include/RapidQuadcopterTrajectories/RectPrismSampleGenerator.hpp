/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by Nathan Bucki <nathan_bucki@berkeley.edu>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <random>
#include "CommonMath/Vec3.hpp"
#include "RapidQuadcopterTrajectories/SampleGenerator.hpp"

namespace RapidQuadrocopterTrajectoryGenerator {

//! Generates uniformly distributed position samples along each axis of a rectangular prism.
class RectPrismSampleGenerator : public SampleGenerator {
 public:

  /*!
   * Note that the prism is aligned with the global coordinate frame.
   *
   * @param sideLen The side lengths of the rectangular prism in the x, y, and z direction
   * @param centerPosition The center of the rectangular prism in global coordinates
   * @param minTime The minimum duration of the trajectory
   * @param maxTime the maximum duration of the trajectory
   */
  RectPrismSampleGenerator(CommonMath::Vec3 sideLen,
                           CommonMath::Vec3 centerPosition, double minTime,
                           double maxTime)
      : _randTime(minTime, maxTime),
        _randX(centerPosition.x - sideLen.x / 2,
               centerPosition.x + sideLen.x / 2),
        _randY(centerPosition.y - sideLen.y / 2,
               centerPosition.y + sideLen.y / 2),
        _randZ(centerPosition.z - sideLen.z / 2,
               centerPosition.z + sideLen.z / 2) {
  }

  //! Returns a 3D position where each component is randomly sampled from a uniform distribution
  CommonMath::Vec3 GetPositionSample() {
    return CommonMath::Vec3(_randX(_generator), _randY(_generator),
                             _randZ(_generator));
  }

  //! Returns a trajectory duration randomly sampled from a uniform distribution
  double GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  std::default_random_engine _generator;
  std::uniform_real_distribution<double> _randTime, _randX, _randY, _randZ;
};
}
