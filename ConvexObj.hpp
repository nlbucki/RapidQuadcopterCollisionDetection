#pragma once
#include "Common/Math/Vec3.hpp"

template<typename Real>
struct Boundary {
  Boundary(Vec3<Real> p, Vec3<Real> n) {
    point = p;
    normal = n;
  }
  Boundary() {
    // If no parameters given, initialize Vec3<Real> using default constructor (nan's for all values)
  }
  Vec3<Real> point;
  Vec3<Real> normal;
};

template<typename Real>
class ConvexObj {
 public:
  virtual ~ConvexObj() {
    // Empty destructor
  }

  virtual Boundary<Real> GetTangentPlane(Vec3<Real> testPoint) = 0;
  virtual bool IsPointInside(Vec3<Real> testPoint) = 0;
  virtual bool IsObstacleInside(Vec3<Real> minCorner, Vec3<Real> maxCorner) = 0;

};
