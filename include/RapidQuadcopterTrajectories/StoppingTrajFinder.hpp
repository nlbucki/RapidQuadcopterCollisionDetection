#pragma once
#include <memory>
#include <vector>
#include <chrono>
#include "CommonMath/Vec3.hpp"
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
#include "CommonMath/Trajectory.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "RapidQuadcopterTrajectories/TrajectoryGenerator.hpp"
#include "RapidQuadcopterTrajectories/SampleGenerator.hpp"

template<class Real>
class StoppingTrajFinder {
 public:
  StoppingTrajFinder(Vec3<Real> pos0, Vec3<Real> vel0, Vec3<Real> acc0,
                     std::shared_ptr<SampleGenerator<Real>> sampleGenerator,
                     Real fmin, Real fmax, Real wmax, Real minTimeSection)
      : _pos0(pos0),
        _vel0(vel0),
        _acc0(acc0),
        _fmin(fmin),
        _fmax(fmax),
        _wmax(wmax),
        _minTimeSection(minTimeSection),
        _dragDefined(false),
        _sampleGenerator(std::move(sampleGenerator)),
        _candidateTraj(
            std::make_shared
                < RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator
                < Real >> (pos0, vel0, acc0, Vec3<Real>(0, 0, -9.81))),
        _checker(_candidateTraj->GetTrajectory()),
        _numTrajGenerated(0),
        _numTrajInputInfeas(0),
        _numFeasTrajGenerated(0) {
    // Candidate trajectories do not consider drag
  }
  void SetObstacles(std::vector<std::shared_ptr<ConvexObj<Real>>> obstacles) {
    _obstacles = obstacles;
  }
  void SetBoundaries(std::vector<Boundary<Real>> boundaries) {
    _boundaries = boundaries;
  }
  void SetTrajectories(std::vector<Trajectory<Real>> trajectories,
                       Real distance) {
    _trajectories = trajectories;
    _minTrajDist = distance;
  }
  bool IsTrajInputFeasible() {
    return _candidateTraj->CheckInputFeasibility(_fmin, _fmax, _wmax,
                                                 _minTimeSection)
        == RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator<Real>::InputFeasible;
  }
  bool IsTrajBoundaryFeasible() {
    // Check boundary constraints
    for (auto boundary : _boundaries) {
      if (_checker.CollisionCheck(boundary)
          != CollisionChecker<Real>::NoCollision) {
        return false;
      }
    }
    return true;
  }
  bool IsTrajObstacleFeasible() {
    for (auto obstacle : _obstacles) {
      if (_checker.CollisionCheck(obstacle, _minTimeSection)
          != CollisionChecker<Real>::NoCollision) {
        return false;
      }
    }
    return true;
  }
  bool IsTrajDynamicallyFeaisble(Vec3<Real> posf) {
    for (auto traj : _trajectories) {
      // First check that our trajectory doesn't collide with the given trajectory
      // Compute relative position
      CollisionChecker<Real> trajChecker(
          _candidateTraj->GetTrajectory() - traj);
      std::shared_ptr<ConvexObj<Real>> sphere = std::make_shared < Sphere
          < Real >> (Vec3<Real>(0, 0, 0), _minTrajDist);
      if (trajChecker.CollisionCheck(sphere, _minTimeSection)
          != CollisionChecker<Real>::NoCollision) {
        return false;
      }
      // Next check that we don't end somewhere that will collide with the other trajectory in the future
      CollisionChecker<Real> endTrajChecker(traj);
      std::shared_ptr<ConvexObj<Real>> endSphere = std::make_shared < Sphere
          < Real >> (posf, _minTrajDist);
      if (endTrajChecker.CollisionCheck(endSphere, _minTimeSection)
          != CollisionChecker<Real>::NoCollision) {
        return false;
      }
    }
    return true;
  }
  // Returns the minimum average jerk trajectory that brings the vehicle to rest.
  // Candidate trajectories are sampled until computationTime (in seconds) is exceeded
  bool GetBestTraj(
      Real computationTime,
      std::shared_ptr<
          RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator<Real>>& outBestTraj) {
    using namespace std::chrono;
    high_resolution_clock::time_point startTime = high_resolution_clock::now();
    Real bestCost = std::numeric_limits<Real>::max();
    bool feasibleTrajFound = false;
    _numTrajGenerated = 0;
    _numFeasTrajGenerated = 0;
    _numTrajInputInfeas = 0;
    while (duration_cast<microseconds>(high_resolution_clock::now() - startTime)
        .count() < int(computationTime * 1e6)) {
      _numTrajGenerated++;
      _candidateTraj->Reset();
      _candidateTraj->SetGoalAcceleration(Vec3<Real>(0, 0, 0));
      _candidateTraj->SetGoalVelocity(Vec3<Real>(0, 0, 0));
      Vec3<Real> posf = _sampleGenerator->GetPositionSample();
      _candidateTraj->SetGoalPosition(posf);
      _candidateTraj->Generate(_sampleGenerator->GetTimeSample());
      Real cost = _candidateTraj->GetCost();
      if (cost > bestCost) {
        // Cost is higher than best cost, skip
        continue;
      }
      _checker = CollisionChecker<Real>(_candidateTraj->GetTrajectory());

      // Check for input + state feasibility
      if (!IsTrajInputFeasible()) {
        // We do not satisfy the bounds on our control inputs
        _numTrajInputInfeas++;
        continue;
      }
      if (!IsTrajObstacleFeasible()) {
        // We intersect with one of the obstacles
        continue;
      }
      if (!IsTrajDynamicallyFeaisble(posf)) {
        // We intersect with one of the given trajectories, or one of the given trajectories intersects with our end position after our trajectory ends
        continue;
      }
      if (!IsTrajBoundaryFeasible()) {
        // We do not satisfy one of the boundary conditions
        continue;
      }

      // We passed all of the input and state feasibility checks, this is a feasible trajectory
      _numFeasTrajGenerated++;  // THIS IS MISLEADING! We only increment this for feasible trajectories of a lower cost than the previously found lowest cost

      // This trajectory is lower cost than the previous trajectory
      bestCost = cost;
      outBestTraj = std::make_shared
          < RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator
          < Real >> (*_candidateTraj);
      feasibleTrajFound = true;
    }
    return feasibleTrajFound;
  }

  int GetNumTrajGenerated() {
    return _numTrajGenerated;
  }
  int GetNumFeasTrajGenerated() {
    return _numFeasTrajGenerated;
  }

  int GetNumInputInfeas() {
    return _numTrajInputInfeas;
  }

  RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator<Real> GetFeasibleTraj();  // TODO: Implement me

 private:
  Vec3<Real> _pos0, _vel0, _acc0;
  Real _fmin, _fmax, _wmax, _minTimeSection, _minTrajDist;
  bool _dragDefined;
  std::shared_ptr<SampleGenerator<Real>> _sampleGenerator;
  std::vector<std::shared_ptr<ConvexObj<Real>>> _obstacles;
  std::vector<Boundary<Real>> _boundaries;
  std::vector<Trajectory<Real>> _trajectories;
  std::shared_ptr<
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator<Real>> _candidateTraj;
  CollisionChecker<Real> _checker;
  int _numTrajGenerated, _numTrajInputInfeas, _numFeasTrajGenerated;
};
