// Represents a quintic polynomial in 3D of the form c[0]*t^5 + c[1]*t^4 + c[2]*t^3 + c[3]*t^2 + c[4]*t + c[5]
// where each coefficient is a 3D vector

#pragma once
#include "Common/Math/Vec3.hpp"
#include "RootFinder/quartic.hpp"
#include <vector>

template<typename Real>
class Trajectory {
 public:
  Trajectory(std::vector<Vec3<Real>> coeffs, Real startTime, Real endTime)
      : _coeffs(coeffs),
        _startTime(startTime),
        _endTime(endTime) {
    assert(coeffs.size() == 6);
    assert(startTime <= endTime);
    boundingBox.initialized = false;
  }
  Trajectory(Vec3<Real> c0, Vec3<Real> c1, Vec3<Real> c2, Vec3<Real> c3,
             Vec3<Real> c4, Vec3<Real> c5, Real startTime, Real endTime)
      : _coeffs { c0, c1, c2, c3, c4, c5 },
        _startTime(startTime),
        _endTime(endTime) {
    assert(startTime <= endTime);
    boundingBox.initialized = false;
  }

  // TODO: Initialize void (use quiet nans)
  // TODO: Initialize with Trajectory object

  Vec3<Real> GetValue(Real t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0] * t * t * t * t * t + _coeffs[1] * t * t * t * t
        + _coeffs[2] * t * t * t + _coeffs[3] * t * t + _coeffs[4] * t
        + _coeffs[5];
  }
  Real GetAxisValue(int i, Real t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0][i] * t * t * t * t * t + _coeffs[1][i] * t * t * t * t
        + _coeffs[2][i] * t * t * t + _coeffs[3][i] * t * t + _coeffs[4][i] * t
        + _coeffs[5][i];
  }

  std::vector<Vec3<Real>> GetCoeffs() {
    return _coeffs;
  }
  Real GetStartTime() const {
    return _startTime;
  }
  Real GetEndTime() const {
    return _endTime;
  }
  Trajectory<Real> GetTimeShiftedTraj(Real shiftTime) {
    Real T2 = shiftTime * shiftTime;
    Real T3 = T2 * shiftTime;
    Real T4 = T3 * shiftTime;
    Real T5 = T4 * shiftTime;
    Vec3<Real> c0 = _coeffs[0];
    Vec3<Real> c1 = 5 * _coeffs[0] * shiftTime + _coeffs[1];
    Vec3<Real> c2 = 10 * _coeffs[0] * T2 + 4 * _coeffs[1] * shiftTime
        + _coeffs[2];
    Vec3<Real> c3 = 10 * _coeffs[0] * T3 + 6 * _coeffs[1] * T2
        + 3 * _coeffs[2] * shiftTime + _coeffs[3];
    Vec3<Real> c4 = 5 * _coeffs[0] * T4 + 4 * _coeffs[1] * T3
        + 3 * _coeffs[2] * T2 + 2 * _coeffs[3] * shiftTime + _coeffs[4];
    Vec3<Real> c5 = _coeffs[0] * T5 + _coeffs[1] * T4 + _coeffs[2] * T3
        + _coeffs[3] * T2 + _coeffs[4] * shiftTime + _coeffs[5];
    return Trajectory<Real>(c0, c1, c2, c3, c4, c5, _startTime - shiftTime,
                            _endTime - shiftTime);
  }

  std::vector<Vec3<Real>> GetDerivativeCoeffs() const {
    std::vector<Vec3<Real>> derivCoeffs;
    derivCoeffs.reserve(5);
    for (int i = 0; i < 5; i++) {
      derivCoeffs.push_back((5 - i) * _coeffs[i]);
    }
    return derivCoeffs;
  }

  void GetBoundingBox(Vec3<Real> &outMinCorner, Vec3<Real> &outMaxCorner) {
    if (!boundingBox.initialized) {
      //calculate the roots of the polynomial in each axis
      Vec3<Real> startVal = GetValue(_startTime);
      Vec3<Real> endVal = GetValue(_endTime);
      for (int dim = 0; dim < 3; dim++) {
        int rootCount;
        Real roots[4];
        if (_coeffs[0][dim]) {
          rootCount = magnet::math::quarticSolve(
              4 * _coeffs[1][dim] / (5 * _coeffs[0][dim]),
              3 * _coeffs[2][dim] / (5 * _coeffs[0][dim]),
              2 * _coeffs[3][dim] / (5 * _coeffs[0][dim]),
              _coeffs[4][dim] / _coeffs[0][dim], roots[0], roots[1], roots[2],
              roots[3]);
        } else {
          rootCount = magnet::math::cubicSolve(
              3 * _coeffs[2][dim] / (4 * _coeffs[1][dim]),
              2 * _coeffs[3][dim] / (4 * _coeffs[1][dim]),
              _coeffs[4][dim] / (4 * _coeffs[1][dim]), roots[0], roots[1],
              roots[2]);
        }
        //Evaluate the acceleration at the boundaries of the period:
        boundingBox.minCorner[dim] = std::min(startVal[dim], endVal[dim]);
        boundingBox.maxCorner[dim] = std::max(startVal[dim], endVal[dim]);

        //Evaluate at the maximum/minimum times:
        for (int i = 0; i < rootCount; i++) {
          if (roots[i] <= _startTime)
            continue;
          if (roots[i] >= _endTime)
            continue;

          boundingBox.minCorner[dim] = std::min(boundingBox.minCorner[dim],
                                                GetAxisValue(dim, roots[i]));
          boundingBox.maxCorner[dim] = std::max(boundingBox.maxCorner[dim],
                                                GetAxisValue(dim, roots[i]));
        }
      }
      boundingBox.initialized = true;
    }
    outMinCorner = boundingBox.minCorner;
    outMaxCorner = boundingBox.maxCorner;
  }

// Returns the difference of two trajectories
// The returned trajectory is only defined at times where this trajectory and rhs are defined
  Trajectory<Real> operator-(const Trajectory<Real> rhs) {
    Real startTime = std::max(_startTime, rhs.GetStartTime());
    Real endTime = std::min(_endTime, rhs.GetEndTime());
    return Trajectory<Real>(_coeffs[0] - rhs[0], _coeffs[1] - rhs[1],
                            _coeffs[2] - rhs[2], _coeffs[3] - rhs[3],
                            _coeffs[4] - rhs[4], _coeffs[5] - rhs[5], startTime,
                            endTime);
  }

  inline Vec3<Real> operator[](int i) const {
    switch (i) {
      case 0:
        return _coeffs[0];
      case 1:
        return _coeffs[1];
      case 2:
        return _coeffs[2];
      case 3:
        return _coeffs[3];
      case 4:
        return _coeffs[4];
      case 5:
        return _coeffs[5];
      default:
        // We should never get here (index out of bounds)
        assert(false);
        return Vec3<Real>();  // Returns vector of NaNs
    }
  }

  void Print() {
    for (int k = 0; k < 6; k++) {
      printf("c%d = (", k);
      for (int i = 0; i < 3; i++) {
        printf("%f, ", _coeffs[k][i]);
      }
      printf("), ");
    }
    printf("\n");
  }

// TODO: Add dot product with direction function

 private:
  std::vector<Vec3<Real>> _coeffs;
  Real _startTime, _endTime;
  struct {
    Vec3<Real> minCorner, maxCorner;
    bool initialized;
  } boundingBox;
};
