#include "SampleGenerator.hpp"
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include <random>

template<typename Real>
class CylinderSampleGenerator : public SampleGenerator<Real> {
  static inline Real tcos(Real in);
  static inline Real tsin(Real in);
  static inline Real tatan2(Real iny, Real inx);
 public:
  CylinderSampleGenerator(Real radius, Real length, Vec3<Real> startPoint,
                          Vec3<Real> direction, Real minTime, Real maxTime)
      : _radius(radius),
        _length(length),
        _minTime(minTime),
        _maxTime(maxTime),
        _startPoint(startPoint),
        _randTime(minTime, maxTime),
        _randRadius(0, radius),
        _randAngle(0, 2*M_PI),
        _randLength(0, length) {
    _direction = direction.GetUnitVector();

    // E = Earth-fixed frame, G = Frame with z-direction pointing along "_direction"
    // Let frame C be an intermediate frame where 3^C = 3^E and 1^C is aligned
    // with the projection of 3^G in the 1^E-2^E plane (i.e. 1^C is on the 1^G-3^G plane)
    Real phi = tatan2(_direction.y, _direction.x);
    Rotation<Real> T_CE = Rotation<Real>::FromAxisAngle(-Vec3<Real>(0, 0, 1),
                                                        phi);
    Real theta = tatan2((T_CE * _direction).x, (T_CE * _direction).z);
    Rotation<Real> T_GC = Rotation<Real>::FromAxisAngle(-Vec3<Real>(0, 1, 0),
                                                        theta);
    _rotation = (T_GC * T_CE).Inverse();
  }
  Vec3<Real> GetPositionSample() {
    Real r = _randRadius(_generator);
    Real theta = _randAngle(_generator);
    Real l = _randLength(_generator);
    Real x = r * tcos(theta);
    Real y = r * tsin(theta);
    Vec3<Real> offset(x, y, l);
    return _startPoint + _rotation * offset;
  }
  Real GetTimeSample() {
    return _randTime(_generator);
  }

 private:
  Real _radius, _length, _minTime, _maxTime;
  Vec3<Real> _startPoint, _direction;
  Rotation<Real> _rotation;  // _rotation*vec rotates vec from the cylinder-fixed frame to the world frame
  std::default_random_engine _generator;
  std::uniform_real_distribution<Real> _randTime, _randRadius, _randAngle,
      _randLength;
};

template<>
inline float CylinderSampleGenerator<float>::tcos(float in) {
  return cosf(in);
}
template<>
inline double CylinderSampleGenerator<double>::tcos(double in) {
  return cos(in);
}
template<>
inline float CylinderSampleGenerator<float>::tsin(float in) {
  return sinf(in);
}
template<>
inline double CylinderSampleGenerator<double>::tsin(double in) {
  return sin(in);
}
template<>
inline float CylinderSampleGenerator<float>::tatan2(float iny, float inx) {
  return atan2f(iny, inx);
}
template<>
inline double CylinderSampleGenerator<double>::tatan2(double iny, double inx) {
  return atan2(iny, inx);
}
