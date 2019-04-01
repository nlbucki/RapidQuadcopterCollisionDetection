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

namespace RapidQuadrocopterTrajectoryGenerator {

//! An abstract class used to generate candidate trajectories that bring the vehicle to rest.
/*!
 *  Classes that inherit from this one should be able to generate candidate end
 *  positions and candidate end times. These will be used to generate a candidate
 *  trajectory from the current state of the vehicle to rest.
 */
class SampleGenerator {
 public:
  virtual ~SampleGenerator() {
    // Empty destructor
  }

  //! Returns the candidate final resting position of the vehicle.
  virtual CommonMath::Vec3 GetPositionSample() = 0;

  //! Returns a candidate duration of the trajectory bringing the vehicle to rest.
  virtual double GetTimeSample() = 0;
};
}
