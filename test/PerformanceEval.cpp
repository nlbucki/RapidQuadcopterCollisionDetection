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
#include <chrono>
#include <random>
#include <fstream>
#include <memory>
#include <vector>
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"

using namespace std;
using namespace std::chrono;
using namespace CommonMath;
using namespace RapidCollisionChecker;
using namespace RapidQuadrocopterTrajectoryGenerator;

/*!
 * This program performs the performance evaluation of the collision detection
 * algorithm as described in Section IV.A. in the associated paper. Candidate
 * trajectories are randomly generated and checked for collisions with a single
 * randomly placed/sized sphere. The average time required to check the candidate
 * trajectory for collisions is computed.
 */
int main(void) {
  // The number of Monte Carlo trials to run. Note that 10^9 trails were used in the
  // associated paper, but here we only run 10^6 so that users can see results quickly.
  int numTrajToGenerate = 1000000;

  // These are the thrust and angular velocity constraints used to reject input-infeasible
  // trajectories.
  double fmin = 5;  // Minimum mass-normalized thrust [m/s^2]
  double fmax = 30;  // Maximum mass-normalized thrust [m/s^2]
  double wmax = 20;  // Maximum angular velocity [rad/s]

  // Minimum time resolution used to check for both input infeasibility and collisions
  double minTimeSec = 0.002;  // [s]

  // Sampling ranges for each axis of trajectory
  double posRange = 4.0;  // Final position [m]
  double velRange = 4.0;  // Initial and final velocity [m/s]
  double accRange = 4.0;  // Initial and final acceleration [m/s^2]
  double timeRangeMin = 0.2;  // Minimum trajectory duration [s]
  double timeRangeMax = 4.0;  // Maximum trajectory duration [s]
  double radiusMin = 0.1;  // Maximum radius of sphere [m]
  double radiusMax = 1.5;  // Minimum radius of sphere [m]

  // Set up random number generator using sampling ranges.
  random_device rd;  // Will be used to obtain a seed for the random number engine
  mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  uniform_real_distribution<> randPos(-posRange, posRange);
  uniform_real_distribution<> randVel(-velRange, velRange);
  uniform_real_distribution<> randAcc(-accRange, accRange);
  uniform_real_distribution<> randTime(timeRangeMin, timeRangeMax);
  uniform_real_distribution<> randRadius(radiusMin, radiusMax);

  // Constants
  Vec3 gravity = Vec3(0, 0, -9.81);  // [m/s^2]
  Vec3 pos0(0, 0, 0);  // Initial position [m]

  // Save settings to a log file
  ofstream monteSim;
  monteSim.open("data/PerformanceEval.txt");  // For storing meta-parameters
  monteSim << "Meta-parameters:" << endl;
  monteSim << "numTrajToGenerate = " << numTrajToGenerate << endl;
  monteSim << "fmin = " << fmin << endl;
  monteSim << "fmax = " << fmax << endl;
  monteSim << "wmax = " << wmax << endl;
  monteSim << "minTimeSec = " << minTimeSec << endl;
  monteSim << "posRange = " << posRange << endl;
  monteSim << "velRange = " << velRange << endl;
  monteSim << "accRange = " << accRange << endl;
  monteSim << "timeRangeMin = " << timeRangeMin << endl;
  monteSim << "timeRangeMax = " << timeRangeMax << endl;
  monteSim << "radiusMin = " << radiusMin << endl;
  monteSim << "radiusMax = " << radiusMax << endl << endl;

  // Counter variables for computing statistics
  int totalTrialsRan = 0;  // This will be larger than numTrajToGenerate because we reject input infeasible trajectories
  int numStateFeasible = 0;
  int numStateInfeasible = 0;
  int numStateIndeterminable = 0;

  // Total time spent performing various operations. These times do not include any time spent performing computation
  // on trajectories that are found to be input infeasible. All are recorded in nanoseconds.
  long nsDefTime = 0;  // Initializing objects, generating random samples, etc.
  long nsGenTime = 0;  // Generating trajectories
  long nsInputFeasTime = 0;  // Checking trajectories for input feasibility
  long nsStateCheckTime = 0;  // Checking trajectories for collisions
  long nsStateFeasCheck = 0;  // Time spent checking trajectories for collisions that end up being collision free
  long nsStateInfeasCheck = 0;  // Time spent checking trajectories for collisions that end up colliding
  long nsStateIndetCheck = 0;  // Time spent checking trajectories for collisions that end up being collision indeterminable

  ///////////////////////////////////////////////////////////////////////////
  // Run the Monte Carlo simulation
  ///////////////////////////////////////////////////////////////////////////
  cout << "Running Monte Carlo simulations for " << numTrajToGenerate
      << " trajectories." << endl;
  for (int simNum = 0; simNum < numTrajToGenerate; simNum++) {
    // Display progress towards completion
    totalTrialsRan++;
    if (totalTrialsRan % 100000 == 0) {
      printf("Percent complete = %.3f%%\n",
             double(simNum) / double(numTrajToGenerate) * 100.0);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Define the start/end states of the trajectory + trajectory duration
    ///////////////////////////////////////////////////////////////////////////
    high_resolution_clock::time_point startDef = high_resolution_clock::now();
    // Initial state:
    Vec3 vel0(randVel(gen), randVel(gen), randVel(gen));  // velocity
    Vec3 acc0(randAcc(gen), randAcc(gen), randAcc(gen));  // acceleration
    // Final state:
    Vec3 posf(randPos(gen), randPos(gen), randPos(gen));  // position
    Vec3 velf(randVel(gen), randVel(gen), randVel(gen));  // velocity
    Vec3 accf(randAcc(gen), randAcc(gen), randAcc(gen));  // acceleration
    // Duration:
    double Tf = randTime(gen);

    // Create trajectory
    RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    traj.SetGoalPosition(posf);
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    // Create randomly placed/sized sphere:
    Vec3 obsPos(randPos(gen), randPos(gen), randPos(gen));
    shared_ptr<ConvexObj> obstacle = make_shared < Sphere
        > (obsPos, randRadius(gen));
    high_resolution_clock::time_point endDef = high_resolution_clock::now();

    ///////////////////////////////////////////////////////////////////////////
    // Generate trajectory
    ///////////////////////////////////////////////////////////////////////////
    high_resolution_clock::time_point startGen = high_resolution_clock::now();
    traj.Generate(Tf);
    high_resolution_clock::time_point endGen = high_resolution_clock::now();

    ///////////////////////////////////////////////////////////////////////////
    // Check input feasibility
    ///////////////////////////////////////////////////////////////////////////
    high_resolution_clock::time_point startInputFeas =
        high_resolution_clock::now();
    RapidTrajectoryGenerator::InputFeasibilityResult inputFeas = traj
        .CheckInputFeasibility(fmin, fmax, wmax, minTimeSec);
    high_resolution_clock::time_point endInputFeas =
        high_resolution_clock::now();
    if (inputFeas
        != RapidTrajectoryGenerator::InputFeasibilityResult::InputFeasible) {
      // This trajectory is not input feasible. Don't record any statistics
      // for this run and start over.
      simNum--;
      continue;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Check state feasibility
    ///////////////////////////////////////////////////////////////////////////
    high_resolution_clock::time_point startStateFeas =
        high_resolution_clock::now();
    CollisionChecker checker(traj.GetTrajectory());
    CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(
        obstacle, minTimeSec);
    high_resolution_clock::time_point endStateFeas =
        high_resolution_clock::now();

    ///////////////////////////////////////////////////////////////////////////
    // Record time spent performing each operation
    ///////////////////////////////////////////////////////////////////////////
    nsDefTime += duration_cast < nanoseconds > (endDef - startDef).count();
    nsGenTime += duration_cast < nanoseconds > (endGen - startGen).count();
    nsInputFeasTime += duration_cast < nanoseconds
        > (endInputFeas - startInputFeas).count();
    long thisCheckTime = duration_cast < nanoseconds
        > (endStateFeas - startStateFeas).count();
    nsStateCheckTime += thisCheckTime;
    switch (stateFeas) {
      case CollisionChecker::NoCollision:
        numStateFeasible++;
        nsStateFeasCheck += thisCheckTime;
        break;
      case CollisionChecker::Collision:
        numStateInfeasible++;
        nsStateInfeasCheck += thisCheckTime;
        break;
      case CollisionChecker::CollisionIndeterminable:
        numStateIndeterminable++;
        nsStateIndetCheck += thisCheckTime;
        break;
    }
  }

  // Print things to console and save to file
  cout << "Done." << endl << endl;
  cout << "Average time spent defining/initializing things [ns] = "
      << double(nsDefTime) / numTrajToGenerate << endl;
  cout << "Average time spent generating the trajectory [ns] = "
      << double(nsGenTime) / numTrajToGenerate << endl;
  cout << "Average time spent checking for input feasibility [ns] = "
      << double(nsInputFeasTime) / numTrajToGenerate << endl;
  cout << "Average time spent checking for collisions [ns] = "
      << double(nsStateCheckTime) / numTrajToGenerate << endl;
  cout
      << "Average time spent checking for collisions when no collision occurs [ns] = "
      << double(nsStateFeasCheck) / numStateFeasible << endl;
  cout
      << "Average time spent checking for collisions when a collision occurs [ns] = "
      << double(nsStateInfeasCheck) / numStateInfeasible << endl;
  cout
      << "Average time spent checking for collisions when a collision cannot be determined [ns] = "
      << double(nsStateIndetCheck) / numStateIndeterminable << endl;
  cout << "Percent of trajectories that were collision-free  = "
      << 100.0 * double(numStateFeasible) / numTrajToGenerate << endl;
  cout << "Percent of trajectories with collision  = "
      << 100.0 * double(numStateInfeasible) / numTrajToGenerate << endl;
  cout << "Percent of trajectories with collision indeterminable  = "
      << 100.0 * double(numStateIndeterminable) / numTrajToGenerate << endl;

  monteSim << "Results:" << endl;
  monteSim << "Average time spent defining/initializing things [ns] = "
      << double(nsDefTime) / numTrajToGenerate << endl;
  monteSim << "Average time spent generating the trajectory [ns] = "
      << double(nsGenTime) / numTrajToGenerate << endl;
  monteSim << "Average time spent checking for input feasibility [ns] = "
      << double(nsInputFeasTime) / numTrajToGenerate << endl;
  monteSim << "Average time spent checking for collisions [ns] = "
      << double(nsStateCheckTime) / numTrajToGenerate << endl;
  monteSim
      << "Average time spent checking for collisions when no collision occurs [ns] = "
      << double(nsStateFeasCheck) / numStateFeasible << endl;
  monteSim
      << "Average time spent checking for collisions when a collision occurs [ns] = "
      << double(nsStateInfeasCheck) / numStateInfeasible << endl;
  monteSim
      << "Average time spent checking for collisions when a collision cannot be determined [ns] = "
      << double(nsStateIndetCheck) / numStateIndeterminable << endl;
  monteSim << "Percent of trajectories that were collision-free  = "
      << 100.0 * double(numStateFeasible) / numTrajToGenerate << endl;
  monteSim << "Percent of trajectories with collision  = "
      << 100.0 * double(numStateInfeasible) / numTrajToGenerate << endl;
  monteSim << "Percent of trajectories with collision indeterminable  = "
      << 100.0 * double(numStateIndeterminable) / numTrajToGenerate << endl;
  monteSim.close();

  return 0;
}

