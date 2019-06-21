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

#include "RapidCollisionDetection/CollisionChecker.hpp"

using namespace CommonMath;
using namespace RapidCollisionChecker;

typename CollisionChecker::CollisionResult CollisionChecker::CollisionCheck(
    CommonMath::Boundary boundary) {
  //Ensure that the normal is a unit vector:
  boundary.normal = boundary.normal.GetUnitVector();

  // Take the dot product of the trajectory with the unit normal
  // This gives the distance of the trajectory from the plane as a function of time
  double c[5] = { 0, 0, 0, 0, 0 };
  std::vector<Vec3> trajDerivativeCoeffs = _traj.GetDerivativeCoeffs();
  for (int dim = 0; dim < 3; dim++) {
    c[0] += boundary.normal[dim] * trajDerivativeCoeffs[0][dim];  //t**4
    c[1] += boundary.normal[dim] * trajDerivativeCoeffs[1][dim];  //t**3
    c[2] += boundary.normal[dim] * trajDerivativeCoeffs[2][dim];  //t**2
    c[3] += boundary.normal[dim] * trajDerivativeCoeffs[3][dim];  //t
    c[4] += boundary.normal[dim] * trajDerivativeCoeffs[4][dim];  //1
  }

  //Solve the roots (we prepend the times 0 and tf):
  double roots[6];
  roots[0] = _traj.GetStartTime();
  roots[1] = _traj.GetEndTime();

  size_t rootCount;
  if (fabs(c[0]) > double(1e-6)) {
    rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots);
  } else {
    rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
  }

  for (unsigned i = 0; i < (rootCount + 2); i++) {
    //don't evaluate points outside the domain
    if (roots[i] < _traj.GetStartTime())
      continue;
    if (roots[i] > _traj.GetEndTime())
      continue;

    if ((_traj.GetValue(roots[i]) - boundary.point).Dot(boundary.normal) <= 0) {
      //touching, or on the wrong side of, the boundary!
      return Collision;
    }
  }
  return NoCollision;
}

typename CollisionChecker::CollisionResult CollisionChecker::CollisionCheck(
    std::shared_ptr<CommonMath::ConvexObj> obstacle, double minTimeSection) {
  // First check if the start or end point of the trajectory is inside the obstacle
  if (obstacle->IsPointInside(_traj.GetValue(_traj.GetStartTime()))
      || obstacle->IsPointInside(_traj.GetValue(_traj.GetEndTime()))) {
    // If the starting or ending point is inside the obstacle, we're infeasible
    return Collision;
  }
  return CollisionCheckSection(_traj.GetStartTime(), _traj.GetEndTime(),
                               obstacle, minTimeSection);
}

typename CollisionChecker::CollisionResult CollisionChecker::CollisionCheckSection(
    double ts, double tf, std::shared_ptr<CommonMath::ConvexObj> obstacle,
    double minTimeSection) {

  // First find the position halfway between the start and end time of this segment
  double midTime = (ts + tf) / 2;
  Vec3 midpoint = _traj.GetValue(midTime);
  if (obstacle->IsPointInside(midpoint)) {
    // The midpoint is inside the obstacle, so we're infeasible
    return Collision;
  }
  if (tf - ts < minTimeSection) {
    // Our time resolution is too small, just give up (trajectory is likely tangent to obstacle surface)
    return CollisionIndeterminable;
  }

  // Get the plane separating the midpoint and the obstacle
  CommonMath::Boundary tangentPlane = obstacle->GetTangentPlane(midpoint);

  // Take the dot product of the trajectory with the unit normal of the separating plane.
  // This gives the distance of the trajectory from the plane as a function of time
  double c[5] = { 0, 0, 0, 0, 0 };
  std::vector<Vec3> trajDerivativeCoeffs = _traj.GetDerivativeCoeffs();
  for (int dim = 0; dim < 3; dim++) {
    c[0] += tangentPlane.normal[dim] * trajDerivativeCoeffs[0][dim];  //t**4
    c[1] += tangentPlane.normal[dim] * trajDerivativeCoeffs[1][dim];  //t**3
    c[2] += tangentPlane.normal[dim] * trajDerivativeCoeffs[2][dim];  //t**2
    c[3] += tangentPlane.normal[dim] * trajDerivativeCoeffs[3][dim];  //t
    c[4] += tangentPlane.normal[dim] * trajDerivativeCoeffs[4][dim];  //1
  }

  // Solve the roots
  double roots[4];
  int rootCount;
  if (fabs(c[0]) > double(1e-6)) {
    rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots);
  } else {
    rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
  }
  // The first "rootCount" entries of roots are now the unordered roots
  std::sort(roots, roots + rootCount);
  // The first "rootCount" entries of roots are now the roots in ascending order

  // Get both lists of points to check (in ascending order)
  std::vector<double> testPointsLow;
  std::vector<double> testPointsHigh;
  testPointsLow.reserve(6);
  testPointsHigh.reserve(6);
  testPointsLow.push_back(ts);  // ts is always the first critical point in testPointsLow
  testPointsHigh.push_back(midTime);  // midTime is always the first critical point in testPointsHigh
  for (int i = 0; i < rootCount; i++) {
    if (roots[i] <= ts) {
      // Skip root if it's before ts
      continue;
    } else if (roots[i] < midTime) {
      // Root is between ts and midTime
      testPointsLow.push_back(roots[i]);
    } else if (roots[i] < tf) {
      // Root is between midTime and tf
      testPointsHigh.push_back(roots[i]);
    } else {
      // Because the roots are in ascending order, there are no more roots are on (ts,tf)
      break;
    }
  }
  testPointsLow.push_back(midTime);  // midTime is always the last critical point in testPointsLow
  testPointsHigh.push_back(tf);  // tf is always the last critical point in testPointsHigh

  // Check testPointsHigh first. We could also check testPointsLow first, but the intuition is
  // that obstacles are more likely to be encountered at later times.
  // We iterate forward in time from midTime to tf.
  for (typename std::vector<double>::iterator it = testPointsHigh.begin() + 1;
      it != testPointsHigh.end(); it++) {
    // Check whether the critical point occurs on the obstacle side of the plane
    if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal)
        <= 0) {
      // This critical point is on the obstacle side of the plane, so we must
      // keep searching over the rest of the trajectory starting at the
      // previous critical point and ending at tf.
      CollisionResult resHigh = CollisionCheckSection(*(it - 1), tf, obstacle,
                                                      minTimeSection);
      if (resHigh == NoCollision) {
        // The section from the previous critical point until tf was feasible, meaning that all of the
        // trajectory from midTime to tf does not collide with the obstacle
        break;
      } else {
        // Either a collision was detected between the previous critical point and tf, or the recursion
        // became too deep (i.e. the time resolution too small) and the collision was indeterminable.
        return resHigh;
      }
    }
  }
  // The segment of the trajectory from midTime to tf is feasible, check the segment of the trajectory
  // from midTime to ts (starting from midTime iterating to ts).
  for (typename std::vector<double>::reverse_iterator it =
      testPointsLow.rbegin() + 1; it != testPointsLow.rend(); it++) {
    // Check whether the critical point occurs on the obstacle side of the plane
    if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal)
        <= 0) {
      // This critical point is on the obstacle side of the plane, so we must
      // keep searching over the rest of the trajectory starting at the
      // previous critical point and ending at ts (recall we are searching
      // backwards in time). Also, because we have already verified that the
      // trajectory is collision free between midTime and tf, we can simply
      // return the result of the collision check on the segment from ts to
      // the previous critical point.
      return CollisionCheckSection(ts, *(it - 1), obstacle, minTimeSection);
    }
  }
  return NoCollision;
}
