#pragma once
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Rotation.hpp"

class Sphere : public ConvexObj {
 public:
  Sphere(Vec3 center, double radius)
      : _centerPoint(center),
        _radius(radius) {
    // Side lengths must be positive
    assert(radius > 0);
  }

  Boundary GetTangentPlane(Vec3 const testPoint) {
    // Rotate into local coordinate frame
    Boundary bound;
    bound.normal = (testPoint - _centerPoint).GetUnitVector();
    bound.point = (testPoint - _centerPoint).GetUnitVector() * _radius
        + _centerPoint;
    return bound;
  }

  bool IsPointInside(Vec3 const testPoint) {
    return (testPoint - _centerPoint).GetNorm2() <= _radius;
  }

  bool IsObstacleInside(Vec3 minCorner, Vec3 maxCorner) {
    // An approximation, obstacles near the corners are erroneously declared inside
    return (_centerPoint.x >= minCorner.x - _radius)
        && (_centerPoint.x <= maxCorner.x + _radius)
        && (_centerPoint.y >= minCorner.y - _radius)
        && (_centerPoint.y <= maxCorner.y + _radius)
        && (_centerPoint.z >= minCorner.z - _radius)
        && (_centerPoint.z <= maxCorner.z + _radius);
  }

 private:
  Vec3 _centerPoint;  // The center in the global coordinate system
  double _radius;

};
