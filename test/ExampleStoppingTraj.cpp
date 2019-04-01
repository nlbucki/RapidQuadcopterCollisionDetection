#include <iostream>
#include <fstream>
#include <chrono>
#include "CommonMath/RectPrism.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "RapidQuadcopterTrajectories/StoppingTrajFinder.hpp"
#include "RapidQuadcopterTrajectories/RectPrismSampleGenerator.hpp"

using namespace std;
using namespace std::chrono;
using namespace CommonMath;
using namespace RapidCollisionChecker;
using namespace RapidQuadrocopterTrajectoryGenerator;

string toCSV(const Vec3 v) {
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}

int main(void) {
  // Set control input limits
  double fmin = 5;  //[m/s**2]
  double fmax = 25;  //[m/s**2]
  double wmax = 20;  //[rad/s]
  double minTimeSec = 0.02;  //[s]

  // Set initial conditions
  Vec3 pos0(0, -2, 1.5);
  Vec3 vel0(0, 6, 0);
  Vec3 acc0(0, 4.9, 0);

  Vec3 sampleSideLengths(5, 5, 5);
  Vec3 sampleCenterPos(0, 0, 0);
  double startSampleTime = 0.2;
  double endSampleTime = 5;
  shared_ptr<SampleGenerator> rectGen = make_shared < RectPrismSampleGenerator
      > (sampleSideLengths, sampleCenterPos, startSampleTime, endSampleTime);

  StoppingTrajFinder stopTraj(pos0, vel0, acc0, rectGen, fmin, fmax, wmax,
                              minTimeSec);

  // Add obstacles
  vector < shared_ptr < ConvexObj >> obstacles;
  Vec3 center(0, 0, 1.5);
  Vec3 sideLengths(0.5, 0.2, 1.5);
  shared_ptr<ConvexObj> rectPrism(
      new RectPrism(center, sideLengths, Rotation::Identity()));
  obstacles.push_back(rectPrism);
  stopTraj.SetObstacles(obstacles);
  shared_ptr<RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator> bestTraj;

  high_resolution_clock::time_point startGen = high_resolution_clock::now();
  bool success = stopTraj.GetBestTraj(1.0, bestTraj);
  high_resolution_clock::time_point endGen = high_resolution_clock::now();
  int measuredCompTime = duration_cast < microseconds
      > (endGen - startGen).count();
  printf("Success = %d\t\tTime = %fs\n", success,
         double(measuredCompTime) / 1.0e6);

  CollisionChecker checker(bestTraj->GetTrajectory());
  CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(
      rectPrism, minTimeSec);
  printf("stateFeas = %s\n",
         CollisionChecker::GetCollisionResultName(stateFeas));
  printf(
      "inputFeas = %s\n",
      RapidTrajectoryGenerator::GetInputFeasibilityResultName(
          bestTraj->CheckInputFeasibility(fmin, fmax, wmax, minTimeSec)));
  double Tf = bestTraj->GetFinalTime();
  printf("EndTime = %f\n", Tf);
  printf("EndPos = (%f, %f, %f)\n", bestTraj->GetPosition(Tf).x,
         bestTraj->GetPosition(Tf).y, bestTraj->GetPosition(Tf).z);

  ofstream obsFile, trajFile;
  obsFile.open("Logs/obstacles.csv");
  trajFile.open("Logs/stoppingTraj.csv");

  obsFile << toCSV(center);
  obsFile << toCSV(sideLengths);

  for (double t = 0; t <= bestTraj->GetFinalTime(); t += 0.01) {
    trajFile << toCSV(bestTraj->GetPosition(t));
    trajFile << '\n';
  }

  obsFile.close();
  trajFile.close();

  return 0;
}

