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

#include "RapidQuadcopterTrajectories/StoppingTrajFinder.hpp"

using namespace CommonMath;
using namespace RapidCollisionChecker;
using namespace RapidQuadrocopterTrajectoryGenerator;

StoppingTrajFinder::StoppingTrajFinder(
    CommonMath::Vec3 pos0, CommonMath::Vec3 vel0, CommonMath::Vec3 acc0,
    std::shared_ptr<SampleGenerator> sampleGenerator, double fmin, double fmax,
    double wmax, double minTimeSection)
    : _pos0(pos0),
      _vel0(vel0),
      _acc0(acc0),
      _fmin(fmin),
      _fmax(fmax),
      _wmax(wmax),
      _minTimeSection(minTimeSection),
      _dragDefined(false),
      _sampleGenerator(std::move(sampleGenerator)),
      _candidateTraj(
          std::make_shared
              < RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator
              > (pos0, vel0, acc0, CommonMath::Vec3(0, 0, -9.81))),
      _checker(_candidateTraj->GetTrajectory()),
      _numTrajGenerated(0) {
}

bool StoppingTrajFinder::GetBestTraj(
    double computationTime,
    std::shared_ptr<
        RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator>& outBestTraj) {
  using namespace std::chrono;
  high_resolution_clock::time_point startTime = high_resolution_clock::now();
  double bestCost = std::numeric_limits<double>::max();
  bool feasibleTrajFound = false;
  _numTrajGenerated = 0;
  while (duration_cast < microseconds
      > (high_resolution_clock::now() - startTime).count()
      < int(computationTime * 1e6)) {
    _numTrajGenerated++;
    _candidateTraj->Reset();
    _candidateTraj->SetGoalAcceleration(Vec3(0, 0, 0));
    _candidateTraj->SetGoalVelocity(Vec3(0, 0, 0));
    Vec3 posf = _sampleGenerator->GetPositionSample();
    _candidateTraj->SetGoalPosition(posf);
    _candidateTraj->Generate(_sampleGenerator->GetTimeSample());
    double cost = _candidateTraj->GetCost();
    if (cost > bestCost) {
      // Cost is higher than best cost, skip
      continue;
    }
    _checker = CollisionChecker(_candidateTraj->GetTrajectory());

    // Check for input + state feasibility
    if (!IsTrajInputFeasible()) {
      // We do not satisfy the bounds on our control inputs
      continue;
    }
    if (!IsTrajBoundaryFeasible()) {
      // We do not satisfy one of the boundary conditions
      continue;
    }
    if (!IsTrajObstacleFeasible()) {
      // We intersect with one of the obstacles
      continue;
    }
    if (!IsTrajDynamicallyFeaisble(posf)) {
      // We intersect with one of the given trajectories, or one of the given trajectories intersects with our end position after our trajectory ends
      continue;
    }

    // We passed all of the input and state feasibility checks, this is a feasible trajectory

    // This trajectory is lower cost than the previous trajectory
    bestCost = cost;
    outBestTraj = std::make_shared
        < RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator
        > (*_candidateTraj);
    feasibleTrajFound = true;
  }
  return feasibleTrajFound;
}

bool StoppingTrajFinder::IsTrajInputFeasible() {
  return _candidateTraj->CheckInputFeasibility(_fmin, _fmax, _wmax,
                                               _minTimeSection)
      == RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator::InputFeasible;
}
bool StoppingTrajFinder::IsTrajBoundaryFeasible() {
  // Check boundary constraints
  for (auto boundary : _boundaries) {
    if (_checker.CollisionCheck(boundary) != CollisionChecker::NoCollision) {
      return false;
    }
  }
  return true;
}
bool StoppingTrajFinder::IsTrajObstacleFeasible() {
  for (auto obstacle : _obstacles) {
    if (_checker.CollisionCheck(obstacle, _minTimeSection)
        != CollisionChecker::NoCollision) {
      return false;
    }
  }
  return true;
}
bool StoppingTrajFinder::IsTrajDynamicallyFeaisble(Vec3 posf) {
  for (auto traj : _trajectories) {
    // First check that our trajectory doesn't collide with the given trajectory
    // Compute relative position
    CollisionChecker trajChecker(_candidateTraj->GetTrajectory() - traj);
    std::shared_ptr<ConvexObj> sphere = std::make_shared < Sphere
        > (Vec3(0, 0, 0), _minTrajDist);
    if (trajChecker.CollisionCheck(sphere, _minTimeSection)
        != CollisionChecker::NoCollision) {
      return false;
    }
    // Next check that we don't end somewhere that will collide with the other trajectory in the future
    CollisionChecker endTrajChecker(traj);
    std::shared_ptr<ConvexObj> endSphere = std::make_shared < Sphere
        > (posf, _minTrajDist);
    if (endTrajChecker.CollisionCheck(endSphere, _minTimeSection)
        != CollisionChecker::NoCollision) {
      return false;
    }
  }
  return true;
}
