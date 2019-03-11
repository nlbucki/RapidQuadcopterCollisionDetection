#pragma once
#include "Trajectory.hpp"
#include "ConvexObj.hpp"
#include <memory>
#include "RootFinder/quartic.hpp"

template<typename Real>
class CollisionChecker {
 public:

  enum CollisionResult {
    NoCollision = 0,
    Collision = 1,
    CollisionIndeterminable = 2,
  };

  CollisionChecker(Trajectory<Real> traj)
      : _traj(traj) {
  }

  // Checks for collision between _traj and half-plane
  CollisionResult CollisionCheck(Boundary<Real> boundary);

  // Checks for collision between _traj and obstacle
  CollisionResult CollisionCheck(std::shared_ptr<ConvexObj<Real>> obstacle,
                                 Real minTimeSection);

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
      Real ts, Real tf, std::shared_ptr<ConvexObj<Real>> obstacle,
      Real minTimeSection);

  Trajectory<Real> _traj;
};
