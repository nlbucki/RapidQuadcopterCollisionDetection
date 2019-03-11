#pragma once
#include "ConvexObj.hpp"
#include "Common/Math/Rotation.hpp"

template<typename Real>
class RectPrism : public ConvexObj<Real> {
  static inline Real tabs(Real in);
 public:
  RectPrism(Vec3<Real> center, Vec3<Real> sideLengths, Rotation<Real> rotation)
      : _centerPoint(center),
        _sideLengths(sideLengths),
        _rotation(rotation),
        _rotationInv(rotation.Inverse()) {
    // Side lengths must be positive
    assert(sideLengths.x > 0 && sideLengths.y > 0 && sideLengths.z > 0);
  }

  Boundary<Real> GetTangentPlane(Vec3<Real> const testPoint) {
    // Rotate into local coordinate frame
    Vec3<Real> p = _rotationInv * (testPoint - _centerPoint);

    // Compute Euclidian projection onto cube
    Boundary<Real> bound;
    for (int i = 0; i < 3; i++) {
      if (p[i] < -_sideLengths[i] / 2) {
        bound.point[i] = -_sideLengths[i] / 2;
      } else if (p[i] > _sideLengths[i] / 2) {
        bound.point[i] = _sideLengths[i] / 2;
      } else {
        bound.point[i] = p[i];
      }
    }

    // Rotate point back to global coordinates and get norm
    bound.point = _rotation * bound.point + _centerPoint;
    bound.normal = (testPoint - bound.point).GetUnitVector();
    return bound;
  }

  bool IsPointInside(Vec3<Real> const testPoint) {
    Vec3<Real> p = _rotationInv * (testPoint - _centerPoint);
    return (tabs(p.x) <= _sideLengths.x / 2)
        && (tabs(p.y) <= _sideLengths.y / 2)
        && (tabs(p.z) <= _sideLengths.z / 2);
  }

  bool IsObstacleInside(Vec3<Real> minCorner, Vec3<Real> maxCorner) {
    // Find sphere that encloses obstacle and use that
    Real radius = (_sideLengths/2).GetNorm2();
    return (_centerPoint.x >= minCorner.x - radius)
        && (_centerPoint.x <= maxCorner.x + radius)
        && (_centerPoint.y >= minCorner.y - radius)
        && (_centerPoint.y <= maxCorner.y + radius)
        && (_centerPoint.z >= minCorner.z - radius)
        && (_centerPoint.z <= maxCorner.z + radius);;
  }
  void SetCenterPoint(Vec3<Real> center){
    _centerPoint = center;
  }
  void SetSideLengths(Vec3<Real> sideLen) {
    _sideLengths = sideLen;
  }
  void SetRotation(Rotation<Real> rot) {
    _rotation = rot;
    _rotationInv = rot.Inverse();
  }

 private:

  Vec3<Real> _centerPoint;  // The center of the prism in the global coordinate system
  Vec3<Real> _sideLengths;  // The length of each side (x, y, z)
  Rotation<Real> _rotation, _rotationInv;  // Rotation from global coordinate system to local coordinate system TODO: Is this true?

};

template<>
inline float RectPrism<float>::tabs(float in) {
  return fabsf(in);
}
template<>
inline double RectPrism<double>::tabs(double in) {
  return fabs(in);
}

