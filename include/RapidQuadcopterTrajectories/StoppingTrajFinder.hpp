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
#include <vector>
#include <chrono>
#include "CommonMath/Vec3.hpp"
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
#include "CommonMath/Trajectory.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "RapidQuadcopterTrajectories/SampleGenerator.hpp"

namespace RapidQuadrocopterTrajectoryGenerator {

//! Class for finding state and input feasible trajectories bringing the vehicle to rest
class StoppingTrajFinder {
 public:

  //! Constructor
  /*!
   * @param pos0 Current position of vehicle [m]
   * @param vel0 Current velocity of vehicle [m/s]
   * @param acc0 Current acceleration of vehicle [m/s^2]
   * @param sampleGenerator SampleGenerator used to generate candidate end positions
   * and trajectory durations
   * @param fminAllowed Minimum thrust value inputs allowed [m/s^2].
   * @param fmaxAllowed Maximum thrust value inputs allowed [m/s^2].
   * @param wmaxAllowed Maximum body rates input allowed [rad/s].
   * @param minTimeSection Minimum time section to test during the recursion [s].
   */
  StoppingTrajFinder(CommonMath::Vec3 pos0, CommonMath::Vec3 vel0,
                     CommonMath::Vec3 acc0,
                     std::shared_ptr<SampleGenerator> sampleGenerator,
                     double fmin, double fmax, double wmax,
                     double minTimeSection);

  //! Sets convex obstacles that should be avoided.
  void SetObstacles(
      std::vector<std::shared_ptr<CommonMath::ConvexObj>> obstacles) {
    _obstacles = obstacles;
  }

  //! Sets planes we should not cross (e.g. the ground).
  void SetBoundaries(std::vector<CommonMath::Boundary> boundaries) {
    _boundaries = boundaries;
  }

  //! Sets dynamic obstacles we should avoid.
  /*!
   * Note that we assume all dynamic obstacles are modeled as spheres. Also, when searching
   * for trajectories that do not collide with the dynamic obstacle, we check that
   * (1) no collisions occur during the candidate trajectory and (2) the candidate trajectory
   * does not end somewhere such that a collision will occur after the vehicle comes to rest.
   *
   * @param trajectories The predicted trajectories of the dynamic obstacles to be avoided
   * @param distance The minimum acceptable distance between the center of the dynamic
   * obstacle and the center of the vehicle (e.g. radius of the vehicle + radius of the obstacle) [m]
   */
  void SetTrajectories(std::vector<CommonMath::Trajectory> trajectories,
                       double distance) {
    _trajectories = trajectories;
    _minTrajDist = distance;
  }

  //! Returns the minimum average jerk trajectory that brings the vehicle to rest without collisions.
  /*
   * This function samples candidate trajectories according to the SampleGenerator passed to the
   * constructor. We first compute the average jerk of the trajectory and reject it if it has a
   * higher average jerk than any previously found state and input feasible trajectory. Next, we
   * check that the total mass-normalized thrust and maximum angular velocity remains bellow
   * within the given bounds. We then check that the candidate trajectory does not cross any
   * of the given boundaries (e.g. walls or the floor). Finally, we check that the candidate
   * trajectory does not collide with any static or dynamic obstacles.
   *
   * @param computationTime Candidate trajectories are sampled until
   * computationTime is exceeded [s]
   * @param outBestTraj The minimum average jerk trajectory that brings the
   * vehicle to rest without collisions.
   * @return true if a state and input feasible stopping trajectory was found, false otherwise
   */
  bool GetBestTraj(
      double computationTime,
      std::shared_ptr<
          RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator>& outBestTraj);

  //! Returns the number of trajectories generated after a call to GetBestTraj()
  int GetNumTrajGenerated() {
    return _numTrajGenerated;
  }

 private:

  bool IsTrajInputFeasible();  //!< Checks trajectory for input feasibility
  bool IsTrajBoundaryFeasible();  //!< Checks trajectory for collisions with given planes
  bool IsTrajObstacleFeasible();  //!< Checks trajectory for collisions with static obstacles
  bool IsTrajDynamicallyFeaisble(CommonMath::Vec3 posf);  //!< Checks trajectory for collisions with dynamic obstacles

  CommonMath::Vec3 _pos0, _vel0, _acc0;
  double _fmin, _fmax, _wmax, _minTimeSection, _minTrajDist;
  bool _dragDefined;
  std::shared_ptr<SampleGenerator> _sampleGenerator;
  std::vector<std::shared_ptr<CommonMath::ConvexObj>> _obstacles;
  std::vector<CommonMath::Boundary> _boundaries;
  std::vector<CommonMath::Trajectory> _trajectories;
  std::shared_ptr<RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator> _candidateTraj;
  RapidCollisionChecker::CollisionChecker _checker;
  int _numTrajGenerated;
};
}
