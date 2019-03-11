/* mwm 
 *
 * A way of representing attitudes, based on the Euler-Rodriguez symmetric parameters (ERSP)
 * a.k.a. "quaternion of rotation". Note that Quaternions as such are stupid and used
 * only by 18th century mathematicians who hadn't yet invented vector maths.
 * If you need to invoke hypercomplex numbers to do classical mechanics, you're doing
 * something wrong.
 *
 * Note that you should call Normalise() every once in a while, so that you're rotations remain
 * orthonormal.
 *
 * BEWARE: Everyone defines rotations differently, take care when interpreting the internals, or
 * using it. Construct some test cases and make sure the numbers make sense.
 *
 */

#pragma once

#include <Common/Math/Vec3.hpp>
#include <Common/Math/Matrix.hpp>
#include <stdio.h>
#include <math.h>
#include <limits>

//Real can be float or double (float for onboard, else double).
template<typename Real>
class Rotation {
  //we define some templated math functions, to ensure we don't accidentally use double versions in float, etc.
  static inline Real tsin(Real in);
  static inline Real tcos(Real in);
  static inline Real tsqrt(Real in);
  static inline Real tasin(Real in);
  static inline Real tatan2(Real in1, Real in2);
  static inline Real tacos(Real in);
  static inline Real tabs_val(Real in);

  Real static constexpr half = Real(0.5);
  Real static constexpr MIN_ANGLE = Real(4.84813681e-6);  //less than one arc second.

 public:
  Rotation(void) {
  }

  //Note: this assumes you're giving it a unit quaternion. Call Normalise() if not.
  Rotation(Real a, Real b, Real c, Real d) {
    _v[0] = a;
    _v[1] = b;
    _v[2] = c;
    _v[3] = d;
  }

  //Copy constructor from Rotationd (for Rotationf)
  Rotation(Rotation<double> const &in) {
    _v[0] = Real(in[0]);
    _v[1] = Real(in[1]);
    _v[2] = Real(in[2]);
    _v[3] = Real(in[3]);
  }

  static inline Rotation Identity(void) {
    return Rotation(1, 0, 0, 0);
  }

  Rotation Inverse(void) const {
    return Rotation(_v[0], -_v[1], -_v[2], -_v[3]);
  }

  //This makes sure we have norm one
  void Normalise(void) {
    Real n = sqrt(
        _v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2] + _v[3] * _v[3]);
    if (n < 1.0e-6) {
      (*this) = Identity();
    } else {
      for (int i = 0; i < 4; i++)
        _v[i] /= n;
    }
  }

  static Rotation FromRotationVector(const Vec3<Real> rotVec) {
    const Real theta = rotVec.GetNorm2();
    if (theta < MIN_ANGLE)
      return Identity();  //less than one arc second :)
    return FromAxisAngle(rotVec / theta, theta);
  }

  //You must pass a unit vector, no check is made here!
  static inline Rotation FromAxisAngle(const Vec3<Real> &unitVector,
                                       const Real &angle) {
    return Rotation(tcos(angle * half), tsin(angle * half) * unitVector.x,
                    tsin(angle * half) * unitVector.y,
                    tsin(angle * half) * unitVector.z);
  }

  static Rotation FromEulerYPR(const Real& y, const Real& p, const Real& r) {  //NB! rotation: 3-2-1 yaw,pitch,roll
    Rotation rot;
    rot[0] = tcos(half * y) * tcos(half * p) * tcos(half * r)
        + tsin(half * y) * tsin(half * p) * tsin(half * r);
    rot[1] = tcos(half * y) * tcos(half * p) * tsin(half * r)
        - tsin(half * y) * tsin(half * p) * tcos(half * r);
    rot[2] = tcos(half * y) * tsin(half * p) * tcos(half * r)
        + tsin(half * y) * tcos(half * p) * tsin(half * r);
    rot[3] = tsin(half * y) * tcos(half * p) * tcos(half * r)
        - tcos(half * y) * tsin(half * p) * tsin(half * r);
    return rot;
  }

  static Rotation FromVectorPartOfQuaternion(const Vec3<Real> in) {
    Real tmp = in.GetNorm2Squared();
    if (tmp > 1.0f)
      return Rotation::Identity();
    Real a0 = sqrt(1 - tmp);
    return Rotation(a0, in.x, in.y, in.z);
  }

  // rotation multiplication: r2*r1, corresponds to a rotation r1 followed by rotation r2
  Rotation operator*(const Rotation& r1) const {
    Real c0 = r1[0] * _v[0] - r1[1] * _v[1] - r1[2] * _v[2] - r1[3] * _v[3];
    Real c1 = r1[1] * _v[0] + r1[0] * _v[1] + r1[3] * _v[2] - r1[2] * _v[3];
    Real c2 = r1[2] * _v[0] - r1[3] * _v[1] + r1[0] * _v[2] + r1[1] * _v[3];
    Real c3 = r1[3] * _v[0] + r1[2] * _v[1] - r1[1] * _v[2] + r1[0] * _v[3];

    return Rotation(c0, c1, c2, c3);
  }

  // rotate a vector forward
  Vec3<Real> operator*(const Vec3<Real>& vec) const {
    return Rotate(vec);
  }

  Real GetAngle(void) const {
//    return tatan2(Vec3<Real>(_v[1], _v[2], _v[3]).GetNorm2(), _v[0]) * Real(2.0);
    Real a = tacos(tabs_val(_v[0])) * Real(2.0);
    return a;
  }

  Vec3<Real> ToRotationVector(void) const {
    const Vec3<Real> n = ToVectorPartOfQuaternion();
    const Real norm = n.GetNorm2();
    const Real angle = tasin(norm) * 2;
    if (angle < MIN_ANGLE) {
      return Vec3<Real>(0, 0, 0);  //less than one arc second
    }

    return n * (angle / norm);
  }

  Vec3<Real> ToVectorPartOfQuaternion(void) const {
    //makes first component positive
    if (_v[0] > 0)
      return Vec3<Real>(_v[1], _v[2], _v[3]);
    else
      return Vec3<Real>(-_v[1], -_v[2], -_v[3]);
  }

  void ToEulerYPR(Real &y, Real &p, Real &r) const {
    y = tatan2(Real(2.0) * _v[1] * _v[2] + Real(2.0) * _v[0] * _v[3],
               _v[1] * _v[1] + _v[0] * _v[0] - _v[3] * _v[3] - _v[2] * _v[2]);
    p = -tasin(Real(2.0) * _v[1] * _v[3] - Real(2.0) * _v[0] * _v[2]);
    r = tatan2(Real(2.0) * _v[2] * _v[3] + Real(2.0) * _v[0] * _v[1],
               _v[3] * _v[3] - _v[2] * _v[2] - _v[1] * _v[1] + _v[0] * _v[0]);
  }

  Vec3<Real> ToEulerYPR(void) const {
    Vec3<Real> out;
    ToEulerYPR(out.x, out.y, out.z);
    return out;
  }

  //for debugging
  void PrintRotationMatrix(void) {
    Real R[9];
    GetRotationMatrix(R);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        printf("%.3f\t", (Real) R[i * 3 + j]);
      }
      printf("\n");
    }
  }

  Real& operator[](unsigned const i) {
    return _v[i];
  }
  const Real& operator[](unsigned const i) const {
    return _v[i];
  }

  void GetRotationMatrix(Real R[9]) const {
    const Real r0 = _v[0] * _v[0];
    const Real r1 = _v[1] * _v[1];
    const Real r2 = _v[2] * _v[2];
    const Real r3 = _v[3] * _v[3];

    /*
     * Matrix =
     *    [[R[0], R[1], R[2]],
     *     [R[3], R[4], R[5]],
     *     [R[6], R[7], R[8]]]
     */

    R[0] = r0 + r1 - r2 - r3;
    R[1] = 2 * _v[1] * _v[2] - 2 * _v[0] * _v[3];
    R[2] = 2 * _v[1] * _v[3] + 2 * _v[0] * _v[2];

    R[3] = 2 * _v[1] * _v[2] + 2 * _v[0] * _v[3];
    R[4] = r0 - r1 + r2 - r3;
    R[5] = 2 * _v[2] * _v[3] - 2 * _v[0] * _v[1];

    R[6] = 2 * _v[1] * _v[3] - 2 * _v[0] * _v[2];
    R[7] = 2 * _v[2] * _v[3] + 2 * _v[0] * _v[1];
    R[8] = r0 - r1 - r2 + r3;
  }

  Matrix<Real, 3, 3> GetRotationMatrix() const {
    Real R[9];
    GetRotationMatrix(R);
    Matrix<Real, 3, 3> rotMat;
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        rotMat(i,j) = R[3*i + j];
      }
    }
    return rotMat;
  }

 private:

  Vec3<Real> Rotate(const Vec3<Real>& in) const {
    Real R[9];
    GetRotationMatrix(R);

    Vec3<Real> ret(R[0] * in.x + R[1] * in.y + R[2] * in.z,
                   R[3] * in.x + R[4] * in.y + R[5] * in.z,
                   R[6] * in.x + R[7] * in.y + R[8] * in.z);

    return ret;
  }

  Vec3<Real> RotateBackwards(const Vec3<Real>& in) const {
    Real R[9];
    Inverse().GetRotationMatrix(R);

    Vec3<Real> ret(R[0] * in.x + R[1] * in.y + R[2] * in.z,
                   R[3] * in.x + R[4] * in.y + R[5] * in.z,
                   R[6] * in.x + R[7] * in.y + R[8] * in.z);

    return ret;
  }

  Real _v[4];
};

template<>
inline float Rotation<float>::tsin(float in) {
  return sinf(in);
}
template<>
inline float Rotation<float>::tcos(float in) {
  return cosf(in);
}
template<>
inline float Rotation<float>::tsqrt(float in) {
  return sqrtf(in);
}
template<>
inline float Rotation<float>::tasin(float in) {
  return asinf(in);
}
template<>
inline float Rotation<float>::tatan2(float in1, float in2) {
  return atan2f(in1, in2);
}
template<>
inline float Rotation<float>::tacos(float in) {
  return acosf(in);
}
template<>
inline float Rotation<float>::tabs_val(float in) {
  return fabsf(in);
}

template<>
inline double Rotation<double>::tsin(double in) {
  return sin(in);
}
template<>
inline double Rotation<double>::tcos(double in) {
  return cos(in);
}
template<>
inline double Rotation<double>::tsqrt(double in) {
  return sqrt(in);
}
template<>
inline double Rotation<double>::tasin(double in) {
  return asin(in);
}
template<>
inline double Rotation<double>::tatan2(double in1, double in2) {
  return atan2(in1, in2);
}
template<>
inline double Rotation<double>::tacos(double in) {
  return acos(in);
}
template<>
inline double Rotation<double>::tabs_val(double in) {
  return fabs(in);
}

//Instances we care about
typedef Rotation<float> Rotationf;
typedef Rotation<double> Rotationd;
