#pragma once
#include "ConvexObj.hpp"
#include "Common/Math/Rotation.hpp"

template<typename Real>
class Sphere : public ConvexObj<Real> {
 public:
  Sphere(Vec3<Real> center, Real radius)
      : _centerPoint(center),
        _radius(radius) {
    // Side lengths must be positive
    assert(radius > 0);
  }

  Boundary<Real> GetTangentPlane(Vec3<Real> const testPoint) {
    // Rotate into local coordinate frame
    Boundary<Real> bound;
    bound.normal = (testPoint - _centerPoint).GetUnitVector();
    bound.point = (testPoint - _centerPoint).GetUnitVector() * _radius
        + _centerPoint;
    return bound;
  }

  bool IsPointInside(Vec3<Real> const testPoint) {
    return (testPoint - _centerPoint).GetNorm2() <= _radius;
  }

  bool IsObstacleInside(Vec3<Real> minCorner, Vec3<Real> maxCorner) {
    // An approximation, obstacles near the corners are erroneously declared inside
    return (_centerPoint.x >= minCorner.x - _radius)
        && (_centerPoint.x <= maxCorner.x + _radius)
        && (_centerPoint.y >= minCorner.y - _radius)
        && (_centerPoint.y <= maxCorner.y + _radius)
        && (_centerPoint.z >= minCorner.z - _radius)
        && (_centerPoint.z <= maxCorner.z + _radius);
  }

 private:
  Vec3<Real> _centerPoint;  // The center in the global coordinate system
  Real _radius;

};
