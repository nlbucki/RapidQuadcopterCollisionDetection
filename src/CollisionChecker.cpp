#include "RapidCollisionDetection/CollisionChecker.hpp"

typename CollisionChecker::CollisionResult CollisionChecker::CollisionCheck(
    Boundary boundary) {
  //Ensure that the normal is a unit vector:
  boundary.normal = boundary.normal.GetUnitVector();

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
    rootCount = magnet::math::quarticSolve(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots[2],
                                           roots[3], roots[4], roots[5]);
  } else {
    rootCount = magnet::math::cubicSolve(c[2] / c[1], c[3] / c[1], c[4] / c[1],
                                         roots[2], roots[3], roots[4]);
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
    std::shared_ptr<ConvexObj> obstacle, double minTimeSection) {
  // First check if the obstacle is in the bounding box of the trajectory
  if (obstacle->IsPointInside(_traj.GetValue(_traj.GetStartTime()))
      || obstacle->IsPointInside(_traj.GetValue(_traj.GetEndTime()))) {
    // If the starting or ending point is inside the obstacle, we're infeasible
    return Collision;
  }
  return CollisionCheckSection(_traj.GetStartTime(), _traj.GetEndTime(),
                               obstacle, minTimeSection);
}

typename CollisionChecker::CollisionResult CollisionChecker::CollisionCheckSection(
    double ts, double tf, std::shared_ptr<ConvexObj> obstacle,
    double minTimeSection) {
  double midTime = (ts + tf) / 2;
  Vec3 midpoint = _traj.GetValue(midTime);
  if (obstacle->IsPointInside(midpoint)) {
    // The midpoint is inside the obstacle
    return Collision;
  }
  if (tf - ts < minTimeSection) {
    // Our stepsize is too small, just give up (trajectory is likely tangent to obstacle surface)
    return CollisionIndeterminable;
  }

  // Get the tangent plane
  Boundary tangentPlane = obstacle->GetTangentPlane(midpoint);

  // Project trajectory into tangent plane
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
    rootCount = magnet::math::quarticSolve(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots[0],
                                           roots[1], roots[2], roots[3]);
  } else {
    rootCount = magnet::math::cubicSolve(c[2] / c[1], c[3] / c[1], c[4] / c[1],
                                         roots[0], roots[1], roots[2]);
  }
  // The first "rootCount" entries of roots are now the unordered roots
  std::sort(roots, roots + rootCount);
  // The first "rootCount" entries of roots are now the roots in ascending order

  // Get both lists of points to check in ascending order first
  std::vector<double> testPointsLow;
  std::vector<double> testPointsHigh;
  testPointsLow.reserve(6);
  testPointsHigh.reserve(6);
  testPointsLow.push_back(ts);
  testPointsHigh.push_back(midTime);
  for (int i = 0; i < rootCount; i++) {
    if (roots[i] <= ts) {
      // Skip root if it's before t1
      continue;
    } else if (roots[i] < midTime) {
      // Root is between t1 and midTime
      testPointsLow.push_back(roots[i]);
    } else if (roots[i] < tf) {
      // Root is between midTime and t2
      testPointsHigh.push_back(roots[i]);
    } else {
      // We're done because the roots are in ascending order
      break;
    }
  }
  testPointsLow.push_back(midTime);
  testPointsHigh.push_back(tf);

  // Check testPointsHigh first. We could also check testPointsLow first, but the intuition is that obstacles are more likely to be encountered at later times.
  // We iterate forward in time from midTime to t2
  for (typename std::vector<double>::iterator it = testPointsHigh.begin() + 1;
      it != testPointsHigh.end(); it++) {
    if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal)
        <= 0) {
      // The time is in the potentially infeasible region
      CollisionResult resHigh = CollisionCheckSection(*(it - 1), tf, obstacle,
                                                      minTimeSection);
      if (resHigh == NoCollision) {
        break;
      } else {
        return resHigh;
      }
    }
  }
  // The upper segment of the trajectory is feasible, check the lower segment (starting from midTime iterating to t1)
  for (typename std::vector<double>::reverse_iterator it =
      testPointsLow.rbegin() + 1; it != testPointsLow.rend(); it++) {
    if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal)
        <= 0) {
      return CollisionCheckSection(ts, *(it - 1), obstacle, minTimeSection);
    }
  }
  return NoCollision;
}
