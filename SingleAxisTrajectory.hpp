/**
 * Rapid trajectory generation for quadrocopters
 *
 *  Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Modified the original files from https://github.com/markwmuller/RapidQuadrocopterTrajectories
 Xiangyu Wu (wuxiangyu@berkeley.edu)

 Changes:
 - used template to make the codes work on MCU
 - merged SingleAxisTrajectory.cpp and SingleAxisTrajectory.hpp into this single file
 */

#pragma once
#include <math.h>
#include <algorithm>//For min/max
#include <limits>//for max double

namespace RapidQuadrocopterTrajectoryGenerator {

//! An axis along one spatial direction
/*!
 * For more information, please refer to Section II of the publication.
 */

//Real can be float or double (float for onboard, else double).
template<typename Real>
class SingleAxisTrajectory {
  static inline Real tpow(Real base, Real power);
  static inline Real tsqrt(Real in);
 public:
  //! Constructor, calls Reset() function.
  SingleAxisTrajectory(void)
      : _a(0),
        _b(0),
        _g(0),
        _cost(std::numeric_limits<Real>::max()) {
    Reset();
  }

  //! Define the trajectory's initial state (position, velocity, acceleration), at time zero
  void SetInitialState(const Real pos0, const Real vel0, const Real acc0) {
    _p0 = pos0;
    _v0 = vel0;
    _a0 = acc0;
    _initJerkDefined = false;
    Reset();
  }

  //! Define the trajectory's initial state (position, velocity, acceleration, jerk), at time zero
  // Using this constructor means that only the final position and final velocity can be specified
  void SetInitialState(const Real pos0, const Real vel0, const Real acc0,
                       const Real jerk0) {
    _p0 = pos0;
    _v0 = vel0;
    _a0 = acc0;
    _j0 = jerk0;
    _initJerkDefined = true;
    Reset();
  }

  //! Define the trajectory's final position (if you don't call this, it is left free)
  void SetGoalPosition(const Real posf) {
    _posGoalDefined = true;
    _pf = posf;
  }

  //! Define the trajectory's final velocity (if you don't call this, it is left free)
  void SetGoalVelocity(const Real velf) {
    _velGoalDefined = true;
    _vf = velf;
  }

  //! Define the trajectory's final acceleration (if you don't call this, it is left free)
  void SetGoalAcceleration(const Real accf) {
    _accGoalDefined = true;
    _af = accf;
  }

  //! Generate the trajectory, from the defined initial state to the defined components of the final state.
  void GenerateTrajectory(const Real Tf) {
    //define starting position:
    Real delta_a = _af - _a0;
    Real delta_v = _vf - _v0 - _a0 * Tf;
    Real delta_p = _pf - _p0 - _v0 * Tf - Real(0.5) * _a0 * Tf * Tf;

    //powers of the end time:
    const Real T2 = Tf * Tf;
    const Real T3 = T2 * Tf;
    const Real T4 = T3 * Tf;
    const Real T5 = T4 * Tf;

    //solve the trajectories, depending on what's constrained:
    if (_initJerkDefined) {
      // TODO: kind of hacky, clean up later
      _a = 20
          * (24 * _p0 - 24 * _pf + 18 * Tf * _v0 + 6 * Tf * _vf + 6 * T2 * _a0
              + T3 * _j0) / T5;
      _b = -4
          * (30 * _p0 - 30 * _pf + 24 * Tf * _v0 + 6 * Tf * _vf + 9 * T2 * _a0
              + 2 * T3 * _j0) / T4;
      _g = _j0;
    } else if (_posGoalDefined && _velGoalDefined && _accGoalDefined) {
      _a = (60 * T2 * delta_a - 360 * Tf * delta_v + 720 * 1 * delta_p) / T5;
      _b = (-24 * T3 * delta_a + 168 * T2 * delta_v - 360 * Tf * delta_p) / T5;
      _g = (3 * T4 * delta_a - 24 * T3 * delta_v + 60 * T2 * delta_p) / T5;
    } else if (_posGoalDefined && _velGoalDefined) {
      _a = (-120 * Tf * delta_v + 320 * delta_p) / T5;
      _b = (72 * T2 * delta_v - 200 * Tf * delta_p) / T5;
      _g = (-12 * T3 * delta_v + 40 * T2 * delta_p) / T5;
    } else if (_posGoalDefined && _accGoalDefined) {
      _a = (-15 * T2 * delta_a + 90 * delta_p) / (2 * T5);
      _b = (15 * T3 * delta_a - 90 * Tf * delta_p) / (2 * T5);
      _g = (-3 * T4 * delta_a + 30 * T2 * delta_p) / (2 * T5);
    } else if (_velGoalDefined && _accGoalDefined) {
      _a = 0;
      _b = (6 * Tf * delta_a - 12 * delta_v) / T3;
      _g = (-2 * T2 * delta_a + 6 * Tf * delta_v) / T3;
    } else if (_posGoalDefined) {
      _a = 20 * delta_p / T5;
      _b = -20 * delta_p / T4;
      _g = 10 * delta_p / T3;
    } else if (_velGoalDefined) {
      _a = 0;
      _b = -3 * delta_v / T3;
      _g = 3 * delta_v / T2;
    } else if (_accGoalDefined) {
      _a = 0;
      _b = 0;
      _g = delta_a / Tf;
    } else {  //Nothing to do!
      _a = _b = _g = 0;
    }

    //Calculate the cost:
    _cost = _g * _g + _b * _g * Tf + _b * _b * T2 / Real(3.0)
        + _a * _g * T2 / Real(3.0) + _a * _b * T3 / Real(4.0)
        + _a * _a * T4 / Real(20.0);
  }

  //! Resets the cost, coefficients, and goal state constraints. Does *not* reset the initial state
  void Reset(void) {
    _posGoalDefined = _velGoalDefined = _accGoalDefined = false;
    _cost = std::numeric_limits<Real>::max();
    _accPeakTimes.initialised = false;
    _posPeakTimes.initialised = false;
  }

  //! Returns the jerk at time t
  Real GetJerk(Real t) const {
    return _g + _b * t + (1 / Real(2.0)) * _a * t * t;
  }

  //! Returns the acceleration at time t
  Real GetAcceleration(Real t) const {
    return _a0 + _g * t + (1 / Real(2.0)) * _b * t * t
        + (1 / Real(6.0)) * _a * t * t * t;
  }

  //! Returns the velocity at time t
  Real GetVelocity(Real t) const {
    return _v0 + _a0 * t + (1 / Real(2.0)) * _g * t * t
        + (1 / Real(6.0)) * _b * t * t * t
        + (1 / Real(24.0)) * _a * t * t * t * t;
  }

  //! Returns the position at time t
  Real GetPosition(Real t) const {
    return _p0 + _v0 * t + (1 / Real(2.0)) * _a0 * t * t
        + (1 / Real(6.0)) * _g * t * t * t
        + (1 / Real(24.0)) * _b * t * t * t * t
        + (1 / Real(120.0)) * _a * t * t * t * t * t;
  }

  //! Calculate the extrema of the acceleration trajectory over a section
  void GetMinMaxAcc(Real &aMinOut, Real &aMaxOut, Real t1, Real t2) {
    if (!_accPeakTimes.initialised) {
      //calculate the roots of the polynomial
      if (_a) {  //solve a quadratic for t
        Real det = _b * _b - 2 * _g * _a;
        if (det < 0) {  //no real roots
          _accPeakTimes.t[0] = 0;
          _accPeakTimes.t[1] = 0;
        } else {
          _accPeakTimes.t[0] = (-_b + tsqrt(det)) / _a;
          _accPeakTimes.t[1] = (-_b - tsqrt(det)) / _a;
        }
      } else {  //solve linear equation: _g + _b*t == 0:
        if (_b)
          _accPeakTimes.t[0] = -_g / _b;
        else
          _accPeakTimes.t[0] = 0;
        _accPeakTimes.t[1] = 0;
      }

      _accPeakTimes.initialised = 1;
    }

    //Evaluate the acceleration at the boundaries of the period:
    aMinOut = std::min(GetAcceleration(t1), GetAcceleration(t2));
    aMaxOut = std::max(GetAcceleration(t1), GetAcceleration(t2));

    //Evaluate at the maximum/minimum times:
    for (int i = 0; i < 2; i++) {
      if (_accPeakTimes.t[i] <= t1)
        continue;
      if (_accPeakTimes.t[i] >= t2)
        continue;

      aMinOut = std::min(aMinOut, GetAcceleration(_accPeakTimes.t[i]));
      aMaxOut = std::max(aMaxOut, GetAcceleration(_accPeakTimes.t[i]));
    }
  }

  void GetMinMaxPos(Real &posMinOut, Real &posMaxOut, Real t1, Real t2) {
    if (!_posPeakTimes.initialised) {
      //calculate the roots of the polynomial
      if (_a) {
        _posPeakTimes.rootCount = magnet::math::quarticSolve(
            4 * _b / _a, 12 * _g / _a, 24 * _a0 / _a, 24 * _v0 / _a,
            _posPeakTimes.t[0], _posPeakTimes.t[1], _posPeakTimes.t[2],
            _posPeakTimes.t[3]);
      } else {
        _posPeakTimes.rootCount = magnet::math::cubicSolve(3 * _g / _b,
                                                           6 * _a0 / _b,
                                                           6 * _v0 / _b,
                                                           _posPeakTimes.t[0],
                                                           _posPeakTimes.t[1],
                                                           _posPeakTimes.t[2]);
      }

      _posPeakTimes.initialised = true;
    }

    //Evaluate the position at the boundaries of the period:
    posMinOut = std::min(GetPosition(t1), GetPosition(t2));
    posMaxOut = std::max(GetPosition(t1), GetPosition(t2));

    //Evaluate at the maximum/minimum times:
    for (int i = 0; i < _posPeakTimes.rootCount; i++) {
      if (_posPeakTimes.t[i] <= t1)
        continue;
      if (_posPeakTimes.t[i] >= t2)
        continue;

      posMinOut = std::min(posMinOut, GetPosition(_posPeakTimes.t[i]));
      posMaxOut = std::max(posMaxOut, GetPosition(_posPeakTimes.t[i]));
    }
  }

  //! Calculate the extrema of the jerk squared over a section
  Real GetMaxJerkSquared(Real t1, Real t2) {
    Real jMaxSqr = std::max(tpow(GetJerk(t1), 2), tpow(GetJerk(t2), 2));

    if (_a) {
      Real tMax = -1;
      tMax = -_b / _a;
      if (tMax > t1 && tMax < t2) {
        jMaxSqr = std::max<Real>(tpow(GetJerk(tMax), 2), jMaxSqr);
      }
    }

    return jMaxSqr;
  }

  //! Get the parameters defining the trajectory
  Real GetParamAlpha(void) const {
    return _a;
  }
  //! Get the parameters defining the trajectory
  Real GetParamBeta(void) const {
    return _b;
  }
  //! Get the parameters defining the trajectory
  Real GetParamGamma(void) const {
    return _g;
  }
  //! Get the parameters defining the trajectory
  Real GetInitialAcceleration(void) const {
    return _a0;
  }
  //! Get the parameters defining the trajectory
  Real GetInitialVelocity(void) const {
    return _v0;
  }
  //! Get the parameters defining the trajectory
  Real GetInitialPosition(void) const {
    return _p0;
  }

  //! Get the trajectory cost value
  Real GetCost(void) const {
    return _cost;
  }

 private:
  Real _p0, _v0, _a0, _j0;  //!< The initial state (position, velocity, acceleration, jerk (optional))
  Real _pf, _vf, _af;  //!< The goal state (position, velocity, acceleration)
  bool _posGoalDefined, _velGoalDefined, _accGoalDefined;  //!< The components of the goal state defined to be fixed (position, velocity, acceleration)
  bool _initJerkDefined;  //!< The initial jerk is fixed, meaning that only final position and final acceleration can be fixed
  Real _a, _b, _g;  //!< The three coefficients that define the trajectory

  Real _cost;  //!< The trajectory cost, J

  struct {
    Real t[2];
    bool initialised;
  } _accPeakTimes;  //!< The times at which the acceleration has minimum/maximum

  struct {
    Real t[4];
    Real rootCount;
    bool initialised;
  } _posPeakTimes;  //!< The times at which the position has minimum/maximum
};

template<>
inline float SingleAxisTrajectory<float>::tpow(float base, float power) {
  return powf(base, power);
}

template<>
inline double SingleAxisTrajectory<double>::tpow(double base, double power) {
  return pow(base, power);
}

template<>
inline float SingleAxisTrajectory<float>::tsqrt(float in) {
  return sqrtf(in);
}

template<>
inline double SingleAxisTrajectory<double>::tsqrt(double in) {
  return sqrt(in);
}

}  //namespace

