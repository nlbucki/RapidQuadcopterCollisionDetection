#pragma once
#include <memory>
#include <algorithm>
#include "CommonMath/Trajectory.hpp"
#include "CommonMath/ConvexObj.hpp"
#include "RootFinder/quartic.hpp"

class CollisionChecker {
 public:

  enum CollisionResult {
    NoCollision = 0,
    Collision = 1,
    CollisionIndeterminable = 2,
  };

  CollisionChecker(Trajectory traj)
      : _traj(traj) {
  }

  // Checks for collision between _traj and half-plane
  CollisionResult CollisionCheck(Boundary boundary);

  // Checks for collision between _traj and obstacle
  CollisionResult CollisionCheck(std::shared_ptr<ConvexObj> obstacle,
                                 double minTimeSection);

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
      double ts, double tf, std::shared_ptr<ConvexObj> obstacle,
      double minTimeSection);

  Trajectory _traj;
};
