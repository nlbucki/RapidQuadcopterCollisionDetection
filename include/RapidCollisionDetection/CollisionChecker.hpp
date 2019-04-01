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
#include <memory>
#include <algorithm>
#include "CommonMath/Trajectory.hpp"
#include "CommonMath/ConvexObj.hpp"
#include "RootFinder/quartic.hpp"

namespace RapidCollisionChecker {

//! A class for detecting collisions between a 3D quintic polynomial trajectory and a convex obstacle.
class CollisionChecker {
 public:

  enum CollisionResult {
    NoCollision = 0,
    Collision = 1,
    CollisionIndeterminable = 2,
  };

  //! Constructor. Requires a Trajectory object.
  CollisionChecker(CommonMath::Trajectory traj)
      : _traj(traj) {
  }

  //! Checks whether or not the trajectory crosses a given plane.
  /*!
   * @param boundary A Boundary struct representing the plane
   * @return Either NoCollision or Collision depending on if the Trajectory crosses the plane
   */
  CollisionResult CollisionCheck(CommonMath::Boundary boundary);

  //! Checks for a collision between the trajectory and a given convex object.
  /*!
   * @param boundary A Boundary struct representing the half-plane
   * @return Either NoCollision or Collision depending on if the Trajectory crosses the plane
   */
  CollisionResult CollisionCheck(
      std::shared_ptr<CommonMath::ConvexObj> obstacle, double minTimeSection);

  static const char* GetCollisionResultName(CollisionResult fr) {
    switch (fr) {
      case CollisionResult::NoCollision:
        return "No Collision";
      case CollisionResult::Collision:
        return "Collision";
      case CollisionResult::CollisionIndeterminable:
        return "CollisionIndeterminable";
    }
    return "Unknown!";
  }

 private:

  CollisionResult CollisionCheckSection(
      double ts, double tf, std::shared_ptr<CommonMath::ConvexObj> obstacle,
      double minTimeSection);

  CommonMath::Trajectory _traj;
};
}
