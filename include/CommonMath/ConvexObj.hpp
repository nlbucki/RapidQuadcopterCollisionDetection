#pragma once
#include "CommonMath/Vec3.hpp"

struct Boundary {
  Boundary(Vec3 p, Vec3 n) {
    point = p;
    normal = n;
  }
  Boundary() {
    // If no parameters given, initialize Vec3 using default constructor (nan's for all values)
  }
  Vec3 point;
  Vec3 normal;
};

class ConvexObj {
 public:
  virtual ~ConvexObj() {
    // Empty destructor
  }

  virtual Boundary GetTangentPlane(Vec3 testPoint) = 0;
  virtual bool IsPointInside(Vec3 testPoint) = 0;
  virtual bool IsObstacleInside(Vec3 minCorner, Vec3 maxCorner) = 0;

};
