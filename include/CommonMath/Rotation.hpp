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
#include <stdio.h>
#include <math.h>
#include <limits>
#include "CommonMath/Vec3.hpp"

//Real can be float or double (float for onboard, else double).
class Rotation {
  double static constexpr half = double(0.5);
  double static constexpr MIN_ANGLE = double(4.84813681e-6);  //less than one arc second.

 public:
  Rotation(void) {
  }

  //Note: this assumes you're giving it a unit quaternion. Call Normalise() if not.
  Rotation(double a, double b, double c, double d) {
    _v[0] = a;
    _v[1] = b;
    _v[2] = c;
    _v[3] = d;
  }

  //Copy constructor from Rotationd (for Rotationf)
  Rotation(Rotation const &in) {
    _v[0] = double(in[0]);
    _v[1] = double(in[1]);
    _v[2] = double(in[2]);
    _v[3] = double(in[3]);
  }

  static inline Rotation Identity(void) {
    return Rotation(1, 0, 0, 0);
  }

  Rotation Inverse(void) const {
    return Rotation(_v[0], -_v[1], -_v[2], -_v[3]);
  }

  //This makes sure we have norm one
  void Normalise(void) {
    double n = sqrt(
        _v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2] + _v[3] * _v[3]);
    if (n < 1.0e-6) {
      (*this) = Identity();
    } else {
      for (int i = 0; i < 4; i++)
        _v[i] /= n;
    }
  }

  static Rotation FromRotationVector(const Vec3 rotVec) {
    const double theta = rotVec.GetNorm2();
    if (theta < MIN_ANGLE)
      return Identity();  //less than one arc second :)
    return FromAxisAngle(rotVec / theta, theta);
  }

  //You must pass a unit vector, no check is made here!
  static inline Rotation FromAxisAngle(const Vec3 &unitVector,
                                       const double &angle) {
    return Rotation(cos(angle * half), sin(angle * half) * unitVector.x,
                    sin(angle * half) * unitVector.y,
                    sin(angle * half) * unitVector.z);
  }

  static Rotation FromEulerYPR(const double& y, const double& p, const double& r) {  //NB! rotation: 3-2-1 yaw,pitch,roll
    Rotation rot;
    rot[0] = cos(half * y) * cos(half * p) * cos(half * r)
        + sin(half * y) * sin(half * p) * sin(half * r);
    rot[1] = cos(half * y) * cos(half * p) * sin(half * r)
        - sin(half * y) * sin(half * p) * cos(half * r);
    rot[2] = cos(half * y) * sin(half * p) * cos(half * r)
        + sin(half * y) * cos(half * p) * sin(half * r);
    rot[3] = sin(half * y) * cos(half * p) * cos(half * r)
        - cos(half * y) * sin(half * p) * sin(half * r);
    return rot;
  }

  static Rotation FromVectorPartOfQuaternion(const Vec3 in) {
    double tmp = in.GetNorm2Squared();
    if (tmp > 1.0f)
      return Rotation::Identity();
    double a0 = sqrt(1 - tmp);
    return Rotation(a0, in.x, in.y, in.z);
  }

  // rotation multiplication: r2*r1, corresponds to a rotation r1 followed by rotation r2
  Rotation operator*(const Rotation& r1) const {
    double c0 = r1[0] * _v[0] - r1[1] * _v[1] - r1[2] * _v[2] - r1[3] * _v[3];
    double c1 = r1[1] * _v[0] + r1[0] * _v[1] + r1[3] * _v[2] - r1[2] * _v[3];
    double c2 = r1[2] * _v[0] - r1[3] * _v[1] + r1[0] * _v[2] + r1[1] * _v[3];
    double c3 = r1[3] * _v[0] + r1[2] * _v[1] - r1[1] * _v[2] + r1[0] * _v[3];

    return Rotation(c0, c1, c2, c3);
  }

  // rotate a vector forward
  Vec3 operator*(const Vec3& vec) const {
    return Rotate(vec);
  }

  double GetAngle(void) const {
//    return tatan2(Vec3(_v[1], _v[2], _v[3]).GetNorm2(), _v[0]) * double(2.0);
    double a = acos(fabs(_v[0])) * double(2.0);
    return a;
  }

  Vec3 ToRotationVector(void) const {
    const Vec3 n = ToVectorPartOfQuaternion();
    const double norm = n.GetNorm2();
    const double angle = asin(norm) * 2;
    if (angle < MIN_ANGLE) {
      return Vec3(0, 0, 0);  //less than one arc second
    }

    return n * (angle / norm);
  }

  Vec3 ToVectorPartOfQuaternion(void) const {
    //makes first component positive
    if (_v[0] > 0)
      return Vec3(_v[1], _v[2], _v[3]);
    else
      return Vec3(-_v[1], -_v[2], -_v[3]);
  }

  void ToEulerYPR(double &y, double &p, double &r) const {
    y = atan2(double(2.0) * _v[1] * _v[2] + double(2.0) * _v[0] * _v[3],
               _v[1] * _v[1] + _v[0] * _v[0] - _v[3] * _v[3] - _v[2] * _v[2]);
    p = -asin(double(2.0) * _v[1] * _v[3] - double(2.0) * _v[0] * _v[2]);
    r = atan2(double(2.0) * _v[2] * _v[3] + double(2.0) * _v[0] * _v[1],
               _v[3] * _v[3] - _v[2] * _v[2] - _v[1] * _v[1] + _v[0] * _v[0]);
  }

  Vec3 ToEulerYPR(void) const {
    Vec3 out;
    ToEulerYPR(out.x, out.y, out.z);
    return out;
  }

  //for debugging
  void PrintRotationMatrix(void) {
    double R[9];
    GetRotationMatrix(R);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        printf("%.3f\t", (double) R[i * 3 + j]);
      }
      printf("\n");
    }
  }

  double& operator[](unsigned const i) {
    return _v[i];
  }
  const double& operator[](unsigned const i) const {
    return _v[i];
  }

  void GetRotationMatrix(double R[9]) const {
    const double r0 = _v[0] * _v[0];
    const double r1 = _v[1] * _v[1];
    const double r2 = _v[2] * _v[2];
    const double r3 = _v[3] * _v[3];

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

 private:

  Vec3 Rotate(const Vec3& in) const {
    double R[9];
    GetRotationMatrix(R);

    Vec3 ret(R[0] * in.x + R[1] * in.y + R[2] * in.z,
                   R[3] * in.x + R[4] * in.y + R[5] * in.z,
                   R[6] * in.x + R[7] * in.y + R[8] * in.z);

    return ret;
  }

  Vec3 RotateBackwards(const Vec3& in) const {
    double R[9];
    Inverse().GetRotationMatrix(R);

    Vec3 ret(R[0] * in.x + R[1] * in.y + R[2] * in.z,
                   R[3] * in.x + R[4] * in.y + R[5] * in.z,
                   R[6] * in.x + R[7] * in.y + R[8] * in.z);

    return ret;
  }

  double _v[4];
};
