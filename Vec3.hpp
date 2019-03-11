/*!
 * Copyright 2016 by Muellerlab, UC Berkeley
 */

#pragma once
#include <assert.h>
#include <math.h>
#include <limits>
#include <cerrno>
#include "Matrix.hpp"

template<typename Real>
class Vec3;

//Some operator overloading (declaration):
// NB: Do not define an instantiation with Real=float and ScalarType=double -- we don't want this implicitly!
template<typename Real, typename ScalarType>
Vec3<Real> operator*(const ScalarType lhs, const Vec3<Real> rhs);
template<typename Real, typename ScalarType>
Vec3<Real> operator*(const Vec3<Real> rhs, const ScalarType lhs);

//! 3D vector class
/*!
 * A 3D vector class, to make passing arguments easier, and allow easy addition etc. of vectors.
 * The template argument Real can be float or double (float for onboard, else double).
 */
template<typename Real>
class Vec3 {
  //we define some templated math functions, to ensure we don't accidentally use double versions in float, etc.
  static inline Real tsqrt(Real in);
 public:
  Real x, y, z;  //!< the three components of the vector

  Vec3(void)
      : x(std::numeric_limits<Real>::quiet_NaN()),
        y(std::numeric_limits<Real>::quiet_NaN()),
        z(std::numeric_limits<Real>::quiet_NaN()) {
  }  //!<Initialises all members to NaN
  Vec3(Real xin, Real yin, Real zin)
      : x(xin),
        y(yin),
        z(zin) {
  }  //!< Initialise vector

  Vec3(const Real in[3])
      : x(in[0]),
        y(in[1]),
        z(in[2]) {
  }  //!< Initialise vector

  Vec3(Vec3<double> const &in)
      : x(Real(in.x)),
        y(Real(in.y)),
        z(Real(in.z)) {
  }  //!< Initialise from Vec3d

  Vec3(Vec3<float> const &in)
      : x(Real(in.x)),
        y(Real(in.y)),
        z(Real(in.z)) {
  }  //!< Initialise from Vec3f

  //!Getter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline Real operator[](int i) const {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    assert(0);
    return std::numeric_limits<Real>::quiet_NaN();
  }

  //!Setter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline Real & operator[](int i) {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    assert(0);
    //fail loudly:
    x = y = z = std::numeric_limits<Real>::quiet_NaN();
    return x;
  }

  //!Calculate the dot product of two vectors
  inline Real Dot(const Vec3 rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  //!Calculate the cross product of two vectors
  inline Vec3 Cross(const Vec3 rhs) const {
    return Vec3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z,
                x * rhs.y - y * rhs.x);
  }

  //!Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
  inline Real GetNorm2Squared(void) const {
    return this->Dot(*this);
  }

  //!Calculate the Euclidean norm of the vector.
  inline Real GetNorm2(void) const {
    return tsqrt(GetNorm2Squared());
  }

  inline Real Test(void) const {
    return x;
  }

  //!Get the unit vector pointing along the same direction as this vector. Will fail for zero vectors.
  inline Vec3 GetUnitVector(void) const {
    float const n = this->GetNorm2();
    return (*this) / n;
  }

  inline Vec3 operator+(const Vec3 rhs) const {
    return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  }
  inline Vec3 operator-(const Vec3 rhs) const {
    return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  inline Vec3 operator/(const Real rhs) const {
    return Vec3(x / rhs, y / rhs, z / rhs);
  }
  inline Vec3 operator+() const {
    return (*this);
  }  //mostly used to write things prettily, contrasting to the below.
  inline Vec3 operator-() const {
    return (*this) * Real(-1);
  }

  inline Vec3 operator+=(const Vec3 rhs) {
    (*this) = (*this) + rhs;
    return *this;
  }
  inline Vec3 operator-=(const Vec3 rhs) {
    (*this) = (*this) - rhs;
    return *this;
  }
  inline Vec3 operator*=(const Real &rhs) {
    *this = (*this) * rhs;
    return *this;
  }
  inline Vec3 operator/=(const Real &rhs) {
    *this = (*this) / rhs;
    return *this;
  }
};

template<>
inline double Vec3<double>::tsqrt(double in) {
  return sqrt(in);
}
template<>
inline float Vec3<float>::tsqrt(float in) {
  return sqrtf(in);
}

//Instances we care about
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;

//Multiply with scalar of same type as Vec3:
template<typename Real>
Vec3<Real> operator*(const Vec3<Real> lhs, const Real rhs) {
  return Vec3<Real>(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
}

template<typename Real>
Vec3<Real> operator*(const Real lhs, const Vec3<Real> rhs) {
  return Vec3<Real>(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

//Multiply with scalar of integer type:
template<typename Real>
Vec3<Real> operator*(const Vec3<Real> lhs, const int rhs) {
  return Vec3<Real>(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
}

template<typename Real>
Vec3<Real> operator*(const int lhs, const Vec3<Real> rhs) {
  return Vec3<Real>(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

//Multiply with Matrix
template<typename Real>
Vec3<Real> operator*(const Matrix<Real, 3,3> lhs, const Vec3<Real> rhs) {
  Vec3<Real> outVec(0,0,0);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      outVec[i] += lhs(i, j) * rhs[j];
    }
  }
  return outVec;
}

