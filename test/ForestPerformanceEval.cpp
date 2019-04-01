#include <iostream>
#include <chrono>
#include <random>
#include <fstream>
#include <memory>
#include <vector>
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/RectPrism.hpp"
#include "CommonMath/Sphere.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"

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
  // Simulation parameters
  int version = 4;
  int numTrees = 5;
  int numSims = 1000000;
  int numTrajToGeneratePerSim = 100;

  // Input limits
  double fmin = 5;  //[m/s**2]
  double fmax = 30;  //[m/s**2]
  double wmax = 20;  //[rad/s]
  double minTimeSec = 0.002;  //[s]

  // Sampling ranges
  double posRange = 2.5;
  double velRangeYZ = 2.0;
  double velRangeXmin = 2.0;
  double velRangeXmax = 8.0;
  double accRangeYZ = 2.0;
  double accRangeXmin = 4.0;
  double accRangeXmax = 10.0;
  double timeRangeMin = 0.5;
  double timeRangeMax = 2.0;
  double treeSideLen = 0.5;
  double treeHeight = 5;

  // Set up random number generator
  random_device rd;  //Will be used to obtain a seed for the random number engine
  mt19937 gen(rd());  //Standard mersenne_twister_engine seeded with rd()
  uniform_real_distribution<> randPos(-posRange, posRange);
  uniform_real_distribution<> randVelYZ(-velRangeYZ, velRangeYZ);
  uniform_real_distribution<> randVelX(velRangeXmin, velRangeXmax);
  uniform_real_distribution<> randAccYZ(-accRangeYZ, accRangeYZ);
  uniform_real_distribution<> randAccX(accRangeXmin, accRangeXmax);
  uniform_real_distribution<> randTime(timeRangeMin, timeRangeMax);

  // Constants
  Vec3 gravity = Vec3(0, 0, -9.81);  //[m/s**2]
  Vec3 pos0(-posRange, 0, 0);  //position
  Vec3 velf(0, 0, 0);  //velocity
  Vec3 accf(0, 0, 0);  //acceleration

  ofstream monteSim, monteReadme, treeParams;
  monteSim.open("Logs/forestSimV" + to_string(version) + ".csv");  // For storing data
  monteReadme.open("Logs/forestSimV" + to_string(version) + ".txt");  // For storing meta-parameters
  treeParams.open("Logs/forestTreeParamsV" + to_string(version) + ".csv");  // For tree data
  monteReadme << "numTrees = " << numTrees << endl;
  monteReadme << "numSims = " << numSims << endl;
  monteReadme << "numTrajToGeneratePerSim = " << numTrajToGeneratePerSim
      << endl;
  monteReadme << "fmin = " << fmin << endl;
  monteReadme << "fmax = " << fmax << endl;
  monteReadme << "wmax = " << wmax << endl;
  monteReadme << "minTimeSec = " << minTimeSec << endl;
  monteReadme << "posRange = " << posRange << endl;
  monteReadme << "velRangeYZ = " << velRangeYZ << endl;
  monteReadme << "velRangeXmin = " << velRangeXmin << endl;
  monteReadme << "velRangeXmax = " << velRangeXmax << endl;
  monteReadme << "accRangeYZ = " << accRangeYZ << endl;
  monteReadme << "accRangeXmin = " << accRangeXmin << endl;
  monteReadme << "accRangeXmax = " << accRangeXmax << endl;
  monteReadme << "timeRangeMin = " << timeRangeMin << endl;
  monteReadme << "timeRangeMax = " << timeRangeMax << endl;
  monteReadme << "treeSideLen = " << treeSideLen << endl;
  monteReadme << "treeHeight = " << treeHeight << endl;

  Vec3 objPos[5] = { Vec3(-1.75, 1.5, 0), Vec3(0.5, -1.5, 0), Vec3(1.5, 0.5, 0),
      Vec3(-1.0, -1.0, 0), Vec3(0.0, 0.8, -0.3) };
  Vec3 objDim(treeSideLen, treeSideLen, treeHeight);
  Rotation objRot[5] = { Rotation::Identity(), Rotation::Identity(),
      Rotation::Identity(), Rotation::FromAxisAngle(
          Vec3(1, 0, 0).GetUnitVector(), 45 * M_PI / 180),
      Rotation::FromAxisAngle(Vec3(1, 0, 0).GetUnitVector(), -45 * M_PI / 180) };

  // Define tree positions
  vector < shared_ptr<ConvexObj> > obstacles;
  for (int i = 0; i < numTrees; i++) {
    treeParams << toCSV(objPos[i]);
    treeParams << toCSV(objDim);
    treeParams << toCSV(objRot[i].ToVectorPartOfQuaternion());
    treeParams << "\n";
    obstacles.push_back(
        make_shared < RectPrism > (objPos[i], objDim, objRot[i]));
  }

  treeParams.close();

  // Counter variables
  int numStateFeasible = 0;
  int numStateInfeasible = 0;
  int numStateIndeterminable = 0;
  int numInputInfeas = 0;
//  long totalDefTime = 0;
  long totalGenTime = 0;
  long totalInputCheckTime = 0;
  long totalStateCheckTime = 0;
  long totalFirstFeasTrajTime = 0;
  long nsAllTrajTime = 0;
  int longestFirstFeasTrajTime = 0;

  for (int simNum = 0; simNum < numSims; simNum++) {
    if (simNum % 1000 == 0) {
      cout << "\tFraction complete: " << double(simNum) / double(numSims)
          << endl;
    }
    //Define the trajectory starting state:
    Vec3 vel0(randVelX(gen), randVelYZ(gen), randVelYZ(gen));  //velocity
    Vec3 acc0(randAccX(gen), randAccYZ(gen), randAccYZ(gen));  //acceleration
    RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    bool firstFeasTrajFound = false;
    high_resolution_clock::time_point firstFeasTrajStart =
        high_resolution_clock::now();
    high_resolution_clock::time_point allTrajStart =
        high_resolution_clock::now();
    for (int trajNum = 0; trajNum < numTrajToGeneratePerSim; trajNum++) {
      high_resolution_clock::time_point startTime =
          high_resolution_clock::now();
      high_resolution_clock::time_point startGen = high_resolution_clock::now();
      //define the goal state:
      Vec3 posf(randPos(gen), randPos(gen), randPos(gen));  //position
      traj.SetGoalPosition(posf);
      //define the duration:
      double Tf = randTime(gen);
      // Generate trajectory
      traj.Generate(Tf);
      high_resolution_clock::time_point endGen = high_resolution_clock::now();
      long nsGenTime = duration_cast < nanoseconds
          > (endGen - startGen).count();
      totalGenTime += nsGenTime;

      // Check input feasibility
      high_resolution_clock::time_point startInputFeas =
          high_resolution_clock::now();
      RapidTrajectoryGenerator::InputFeasibilityResult inputFeas = traj
          .CheckInputFeasibility(fmin, fmax, wmax, minTimeSec);
      high_resolution_clock::time_point endInputFeas =
          high_resolution_clock::now();
      long nsInputFeasTime = duration_cast < nanoseconds
          > (endInputFeas - startInputFeas).count();
      totalInputCheckTime += nsInputFeasTime;
      if (inputFeas != RapidTrajectoryGenerator::InputFeasible) {
        numInputInfeas++;
      }

      // Check state feasibility
      high_resolution_clock::time_point startStateFeas =
          high_resolution_clock::now();
      CollisionChecker::CollisionResult stateFeas =
          CollisionChecker::NoCollision;
      for (auto obs : obstacles) {
        CollisionChecker checker(traj.GetTrajectory());
        stateFeas = checker.CollisionCheck(obs, minTimeSec);
        if (stateFeas != CollisionChecker::NoCollision) {
          break;
        }
      }
      high_resolution_clock::time_point endStateFeas =
          high_resolution_clock::now();
      long nsStateCheckTime = duration_cast < nanoseconds
          > (endStateFeas - startStateFeas).count();
      totalStateCheckTime += nsStateCheckTime;
      switch (stateFeas) {
        case RapidTrajectoryGenerator::StateFeasible:
          numStateFeasible++;
          if (!firstFeasTrajFound) {
            firstFeasTrajFound = true;
            high_resolution_clock::time_point firstFeasTrajEnd =
                high_resolution_clock::now();
            int duration = duration_cast < nanoseconds
                > (firstFeasTrajEnd - firstFeasTrajStart).count();
            totalFirstFeasTrajTime += duration;
            if (duration > longestFirstFeasTrajTime) {
              longestFirstFeasTrajTime = duration;
            }
          }
          break;
        case RapidTrajectoryGenerator::StateInfeasible:
          numStateInfeasible++;
          break;
        case RapidTrajectoryGenerator::StateIndeterminable:
          numStateIndeterminable++;
          break;
      }
      high_resolution_clock::time_point endTime = high_resolution_clock::now();
      auto nsDuration = duration_cast < nanoseconds
          > (endTime - startTime).count();

      if (simNum == 0) {
        monteSim << toCSV(pos0);
        monteSim << toCSV(vel0);
        monteSim << toCSV(acc0);
        monteSim << toCSV(posf);
        monteSim << toCSV(velf);
        monteSim << toCSV(accf);
        monteSim << Tf << ",";
        monteSim << inputFeas << ",";
        monteSim << stateFeas << ",";
        monteSim << nsDuration << ",";
        monteSim << nsGenTime << ",";
        monteSim << nsInputFeasTime << ",";
        monteSim << nsStateCheckTime << ",";
        monteSim << "\n";
      }
    }
    high_resolution_clock::time_point allTrajEnd = high_resolution_clock::now();
    nsAllTrajTime += duration_cast < nanoseconds
        > (allTrajEnd - allTrajStart).count();
    if (!firstFeasTrajFound) {
      cout << "NO FEASIBLE TRAJECTORIES FOUND!" << endl;
    }
    if (simNum == 0) {
      monteSim.close();
    }
  }

  monteReadme << endl;
  monteReadme << "\tAverage gen time [ns] = "
      << ((double) totalGenTime) / numTrajToGeneratePerSim / numSims << endl;
  monteReadme << "\tAverage input feasibility check time [ns] = "
      << ((double) totalInputCheckTime) / numTrajToGeneratePerSim / numSims
      << endl;
  monteReadme << "\tAverage state feasibility check time [ns] = "
      << ((double) totalStateCheckTime) / numTrajToGeneratePerSim / numSims
      << endl;
  monteReadme << "\tAverage state feasibility check time per obstacle [ns] = "
      << ((double) totalStateCheckTime) / numTrajToGeneratePerSim / numTrees
          / numSims << endl;
  monteReadme << "\tAverage time to find first feasible trajectory [ns] = "
      << ((double) totalFirstFeasTrajTime) / numSims << endl;
  monteReadme << "\tLongest time to find feasible trajectory [ns] = "
      << longestFirstFeasTrajTime << endl << endl;
  monteReadme << "\tAverage time to do everything per trajectory [ns] = "
      << nsAllTrajTime / numSims / numTrajToGeneratePerSim << endl << endl;

  monteReadme << "\tPercent state feasible = "
      << 100.0 * ((double) numStateFeasible)
          / (numTrajToGeneratePerSim * numSims) << endl << endl;

  monteReadme.close();
  cout << "Done." << endl;

  return 0;
}

