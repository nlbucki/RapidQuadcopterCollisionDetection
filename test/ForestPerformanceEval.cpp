/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by Nathan Bucki <nathan_bucki@berkeley.edu>
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

#include <iostream>
#include <sstream>
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
string toCSV(const double R[9]) {
  stringstream ss;
  for (int i = 0; i < 9; i++) {
    ss << R[i] << ",";
  }
  return ss.str();
}

/*!
 * This program performs the performance evaluation of the collision detection
 * algorithm as described in Section IV.B. in the associated paper. Many batches
 * of trajectories starting at a given initial state and ending at a randomly
 * chosen final state are generated and checked for collisions with static
 * obstacles. A single batch of trajectories can be visualized by running
 * plotSingleForestBatch.py in the scripts/ folder.
 */
int main(void) {
  // The number of Monte Carlo trials to run. Note that 10^6 batches were used in the
  // associated paper, but here we only run 10^4 batches so that users can see results quickly.
  int numSims = 10000;
  int numTrajToGeneratePerSim = 100;  // Number of trajectories generated per batch
  int numTrees = 5;  // Number of obstacles

  // These are the thrust and angular velocity constraints used to reject input-infeasible
  // trajectories.
  double fmin = 5;  // Minimum mass-normalized thrust [m/s^2]
  double fmax = 30;  // Maximum mass-normalized thrust [m/s^2]
  double wmax = 20;  // Maximum angular velocity [rad/s]

  // Minimum time resolution used to check for both input infeasibility and collisions
  double minTimeSec = 0.002;  // [s]

  // Sampling ranges
  double posRange = 2.5;  // Final position [m]
  double velRangeYZ = 2.0;  // Initial velocity in y- and z-direction [m/s]
  double velRangeXmin = 2.0;  // Minimum initial velocity in x-direction [m/s]
  double velRangeXmax = 8.0;  // Maximum initial velocity in x-direction [m/s]
  double accRangeYZ = 2.0;  // Initial acceleration in y- and z-direction [m/s^2]
  double accRangeXmin = 4.0;  // Minimum initial acceleration in x-direction [m/s^2]
  double accRangeXmax = 10.0;  // Maximum initial acceleration in x-direction [m/s^2]
  double timeRangeMin = 0.5;  // Minimum trajectory duration [s]
  double timeRangeMax = 2.0;  // Maximum trajectory duration [s]

  // Set up random number generator using sampling ranges.
  mt19937 gen(0);  // Use a constant seed so that trials are repeatable
  uniform_real_distribution<> randPos(-posRange, posRange);
  uniform_real_distribution<> randVelYZ(-velRangeYZ, velRangeYZ);
  uniform_real_distribution<> randVelX(velRangeXmin, velRangeXmax);
  uniform_real_distribution<> randAccYZ(-accRangeYZ, accRangeYZ);
  uniform_real_distribution<> randAccX(accRangeXmin, accRangeXmax);
  uniform_real_distribution<> randTime(timeRangeMin, timeRangeMax);

  // Constants
  Vec3 gravity = Vec3(0, 0, -9.81);  //[m/s^2]
  Vec3 pos0(-posRange, 0, 0);  // Initial position [m]
  Vec3 velf(0, 0, 0);  // Final velocity [m/s]
  Vec3 accf(0, 0, 0);  // Final acceleration [m/s^2]
  double treeSideLen = 0.5;  // Width/length of rectangular prisms ("trees")
  double treeHeight = 5;  // Height of rectangular prisms

  // Save settings to log files
  ofstream monteTxt, monteCSV, treeParamsCSV;
  monteTxt.open("data/ForestPerfEval.txt");  // For storing meta-parameters
  monteCSV.open("data/ForestPerfEval.csv");  // For storing data from one batch of trajectories
  treeParamsCSV.open("data/ForestPerfEvaltreeParamsCSV.csv");  // For tree data
  monteTxt << "Meta-parameters:" << endl;
  monteTxt << "numTrees = " << numTrees << endl;
  monteTxt << "numSims = " << numSims << endl;
  monteTxt << "numTrajToGeneratePerSim = " << numTrajToGeneratePerSim << endl;
  monteTxt << "fmin = " << fmin << endl;
  monteTxt << "fmax = " << fmax << endl;
  monteTxt << "wmax = " << wmax << endl;
  monteTxt << "minTimeSec = " << minTimeSec << endl;
  monteTxt << "posRange = " << posRange << endl;
  monteTxt << "velRangeYZ = " << velRangeYZ << endl;
  monteTxt << "velRangeXmin = " << velRangeXmin << endl;
  monteTxt << "velRangeXmax = " << velRangeXmax << endl;
  monteTxt << "accRangeYZ = " << accRangeYZ << endl;
  monteTxt << "accRangeXmin = " << accRangeXmin << endl;
  monteTxt << "accRangeXmax = " << accRangeXmax << endl;
  monteTxt << "timeRangeMin = " << timeRangeMin << endl;
  monteTxt << "timeRangeMax = " << timeRangeMax << endl;
  monteTxt << "treeSideLen = " << treeSideLen << endl;
  monteTxt << "treeHeight = " << treeHeight << endl << endl;

  // Constants defining the "trees" (modeled as rectangular prisms)
  Vec3 objPos[5] = { Vec3(-1.75, 1.5, 0), Vec3(0.5, -1.5, 0), Vec3(1.5, 0.5, 0),
      Vec3(-1.0, -1.0, 0), Vec3(0.0, 0.8, -0.3) };
  Vec3 objDim(treeSideLen, treeSideLen, treeHeight);
  Rotation objRot[5] = { Rotation::Identity(), Rotation::Identity(),
      Rotation::Identity(), Rotation::FromAxisAngle(
          Vec3(1, 0, 0).GetUnitVector(), 45 * M_PI / 180),
      Rotation::FromAxisAngle(Vec3(1, 0, 0).GetUnitVector(), -45 * M_PI / 180) };

  // Create the trees/obstacles
  vector < shared_ptr<ConvexObj> > obstacles;
  for (int i = 0; i < numTrees; i++) {
    treeParamsCSV << toCSV(objPos[i]);
    treeParamsCSV << toCSV(objDim);
    double R[9];
    objRot[i].GetRotationMatrix(R);
    treeParamsCSV << toCSV(R);
    treeParamsCSV << "\n";
    obstacles.push_back(
        make_shared < RectPrism > (objPos[i], objDim, objRot[i]));
  }
  treeParamsCSV.close();

  // Counter variables for computing statistics
  int numStateFeasible = 0;
  int numInputFeasible = 0;

  // Total time spent performing various operations. These times do not include any time spent performing computation
  // on trajectories that are found to be input infeasible. All are recorded in nanoseconds.
  long nsGenTime = 0;  // Generating trajectories
  long nsInputCheckTime = 0;  // Checking trajectories for input feasibility
  long nsStateCheckTime = 0;  // Checking trajectories for collisions
  long nsFirstFeasTrajTime = 0;  // Time spent searching for the first feasible trajectory of the batch
  long nsAllTrajTime = 0;  // Time spent doing everything

  // Run the Monte Carlo simulation
  cout << "Running Monte Carlo simulations for " << numSims << " batches with "
      << numTrajToGeneratePerSim << " trajectories each." << endl;
  for (int simNum = 0; simNum < numSims; simNum++) {
    // Display progress towards completion
    if (simNum % 1000 == 0) {
      printf("Percent complete = %.3f%%\n",
             double(simNum) / double(numSims) * 100.0);
    }

    // Define the start state of the vehicle for this batch
    Vec3 vel0(randVelX(gen), randVelYZ(gen), randVelYZ(gen));  // velocity
    Vec3 acc0(randAccX(gen), randAccYZ(gen), randAccYZ(gen));  // acceleration
    RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    // Define the end state to be at rest
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    // Simulate the next batch of trajectories
    bool firstFeasTrajFound = false;
    high_resolution_clock::time_point allTrajStart =
        high_resolution_clock::now();
    for (int trajNum = 0; trajNum < numTrajToGeneratePerSim; trajNum++) {

      // Randomly sample a final resting position
      Vec3 posf(randPos(gen), randPos(gen), randPos(gen));
      traj.SetGoalPosition(posf);

      // Randomly sample a trajectory duration
      double Tf = randTime(gen);

      // Generate trajectory
      high_resolution_clock::time_point startGen = high_resolution_clock::now();
      traj.Generate(Tf);
      high_resolution_clock::time_point endGen = high_resolution_clock::now();

      // Check input feasibility
      high_resolution_clock::time_point startInputFeas =
          high_resolution_clock::now();
      RapidTrajectoryGenerator::InputFeasibilityResult inputFeas = traj
          .CheckInputFeasibility(fmin, fmax, wmax, minTimeSec);
      high_resolution_clock::time_point endInputFeas =
          high_resolution_clock::now();
      if (inputFeas == RapidTrajectoryGenerator::InputFeasible) {
        numInputFeasible++;
      }

      // Check for collisions
      high_resolution_clock::time_point startStateFeas =
          high_resolution_clock::now();
      CollisionChecker::CollisionResult stateFeas =
          CollisionChecker::NoCollision;
      for (auto obs : obstacles) {
        CollisionChecker checker(traj.GetTrajectory());
        stateFeas = checker.CollisionCheck(obs, minTimeSec);
        if (stateFeas != CollisionChecker::NoCollision) {
          // This treats CollisionIndeterminable as a collision.
          // Since a collision with an obstacle was detected, we can stop checking
          // the other obstacles for collisions.
          break;
        }
      }
      high_resolution_clock::time_point endStateFeas =
          high_resolution_clock::now();
      if (stateFeas == CollisionChecker::NoCollision) {
        numStateFeasible++;
      }

      // Record time spent performing each operation
      long nsGenTimeThis = duration_cast < nanoseconds
          > (endGen - startGen).count();  // Time spent generating this trajectory
      nsGenTime += nsGenTimeThis;
      long nsInputCheckTimeThis = duration_cast < nanoseconds
          > (endInputFeas - startInputFeas).count();  // Time spent checking this trajectory for input feasibility
      nsInputCheckTime += nsInputCheckTimeThis;
      long nsStateCheckTimeThis = duration_cast < nanoseconds
          > (endStateFeas - startStateFeas).count();  // Time spent checking this trajectory for collisions
      nsStateCheckTime += nsStateCheckTimeThis;

      // Check to see if this is the first collision-free trajectory found of the batch
      if (!firstFeasTrajFound && stateFeas == CollisionChecker::NoCollision) {
        // Record how long it took to find the first feasible trajectory
        high_resolution_clock::time_point firstFeasTrajEnd =
            high_resolution_clock::now();
        int duration = duration_cast < nanoseconds
            > (firstFeasTrajEnd - allTrajStart).count();
        nsFirstFeasTrajTime += duration;
        firstFeasTrajFound = true;
      }

      // Only record trajectory-specific data for the first batch to save memory/time
      if (simNum == 0) {
        Trajectory t = traj.GetTrajectory();
        for (int i = 0; i < 6; i++) {
          monteCSV << toCSV(t[i]);
        }
        monteCSV << Tf << ",";
        monteCSV << inputFeas << ",";
        monteCSV << stateFeas << ",";
        monteCSV << nsGenTimeThis << ",";
        monteCSV << nsInputCheckTimeThis << ",";
        monteCSV << nsStateCheckTimeThis << ",";
        monteCSV << "\n";
      }
    }
    // Record how long the batch took to run. Note this is a bit of an overestimate because
    // it includes the time spent dealing with all of the internal timing, counters, etc. of
    // the batch.
    high_resolution_clock::time_point allTrajEnd = high_resolution_clock::now();
    nsAllTrajTime += duration_cast < nanoseconds
        > (allTrajEnd - allTrajStart).count();
    if (!firstFeasTrajFound) {
      // This should be rare
      cout << "NO FEASIBLE TRAJECTORIES FOUND!" << endl;
    }
    if (simNum == 0) {
      // After the data for each trajectory in the first batch is recorded, close the file.
      monteCSV.close();
    }
  }

  // Print things to console and save to file
  cout << "Done." << endl << endl;
  cout << "Average time spent generating the trajectory [ns] = "
      << double(nsGenTime) / numTrajToGeneratePerSim / numSims << endl;
  cout << "Average time spent checking for input feasibility [ns] = "
      << double(nsInputCheckTime) / numTrajToGeneratePerSim / numSims << endl;
  cout << "Average time spent checking for collisions [ns] = "
      << double(nsStateCheckTime) / numTrajToGeneratePerSim / numSims << endl;
  cout << "Average time spent checking for collisions per obstacle [ns] = "
      << double(nsStateCheckTime) / numTrajToGeneratePerSim / numSims / numTrees
      << endl;
  cout << "Average time to find first collision-free trajectory [ns] = "
      << double(nsFirstFeasTrajTime) / numSims << endl;
  cout << "Average total computation time per trajectory [ns] = "
      << double(nsAllTrajTime) / numSims / numTrajToGeneratePerSim << endl;
  cout << "Percent of trajectories that were input feasible = "
      << 100.0 * double(numInputFeasible) / numSims / numTrajToGeneratePerSim
      << endl;
  cout << "Percent of trajectories that were collision free = "
      << 100.0 * double(numStateFeasible) / numSims / numTrajToGeneratePerSim
      << endl;

  monteTxt << "Results:" << endl;
  monteTxt << "Average time spent generating the trajectory [ns] = "
      << double(nsGenTime) / numTrajToGeneratePerSim / numSims << endl;
  monteTxt << "Average time spent checking for input feasibility [ns] = "
      << double(nsInputCheckTime) / numTrajToGeneratePerSim / numSims << endl;
  monteTxt << "Average time spent checking for collisions [ns] = "
      << double(nsStateCheckTime) / numTrajToGeneratePerSim / numSims << endl;
  monteTxt << "Average time spent checking for collisions per obstacle [ns] = "
      << double(nsStateCheckTime) / numTrajToGeneratePerSim / numSims / numTrees
      << endl;
  monteTxt << "Average time to find first collision-free trajectory [ns] = "
      << double(nsFirstFeasTrajTime) / numSims << endl;
  monteTxt << "Average total computation time per trajectory [ns] = "
      << double(nsAllTrajTime) / numSims / numTrajToGeneratePerSim << endl;
  monteTxt << "Percent of trajectories that were input feasible = "
      << 100.0 * double(numInputFeasible) / numSims / numTrajToGeneratePerSim
      << endl;
  monteTxt << "Percent of trajectories that were collision free = "
      << 100.0 * double(numStateFeasible) / numSims / numTrajToGeneratePerSim
      << endl;

  return 0;
}

