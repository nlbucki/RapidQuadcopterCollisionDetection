#pragma once
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Rotation.hpp"

class RectPrism : public ConvexObj {
 public:
  RectPrism(Vec3 center, Vec3 sideLengths, Rotation rotation)
      : _centerPoint(center),
        _sideLengths(sideLengths),
        _rotation(rotation),
        _rotationInv(rotation.Inverse()) {
    // Side lengths must be positive
    assert(sideLengths.x > 0 && sideLengths.y > 0 && sideLengths.z > 0);
  }

  Boundary GetTangentPlane(Vec3 const testPoint) {
    // Rotate into local coordinate frame
    Vec3 p = _rotationInv * (testPoint - _centerPoint);

    // Compute Euclidian projection onto cube
    Boundary bound;
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

  bool IsPointInside(Vec3 const testPoint) {
    Vec3 p = _rotationInv * (testPoint - _centerPoint);
    return (fabs(p.x) <= _sideLengths.x / 2)
        && (fabs(p.y) <= _sideLengths.y / 2)
        && (fabs(p.z) <= _sideLengths.z / 2);
  }

  bool IsObstacleInside(Vec3 minCorner, Vec3 maxCorner) {
    // Find sphere that encloses obstacle and use that
    double radius = (_sideLengths/2).GetNorm2();
    return (_centerPoint.x >= minCorner.x - radius)
        && (_centerPoint.x <= maxCorner.x + radius)
        && (_centerPoint.y >= minCorner.y - radius)
        && (_centerPoint.y <= maxCorner.y + radius)
        && (_centerPoint.z >= minCorner.z - radius)
        && (_centerPoint.z <= maxCorner.z + radius);;
  }
  void SetCenterPoint(Vec3 center){
    _centerPoint = center;
  }
  void SetSideLengths(Vec3 sideLen) {
    _sideLengths = sideLen;
  }
  void SetRotation(Rotation rot) {
    _rotation = rot;
    _rotationInv = rot.Inverse();
  }

 private:

  Vec3 _centerPoint;  // The center of the prism in the global coordinate system
  Vec3 _sideLengths;  // The length of each side (x, y, z)
  Rotation _rotation, _rotationInv;  // Rotation from global coordinate system to local coordinate system TODO: Is this true?

};

