#include <iostream>
#include <chrono>
#include <random>
#include <fstream>
#include <memory>
#include <vector>
#include "Components/TrajectoryGenerator/TrajectoryGenerator.hpp"
#include "Components/TrajectoryGenerator/ConvexObj.hpp"
#include "Components/TrajectoryGenerator/RectPrism.hpp"
#include "Components/TrajectoryGenerator/Sphere.hpp"
#include "Components/TrajectoryGenerator/CollisionChecker.hpp"

using namespace std;
using namespace std::chrono;
using namespace RapidQuadrocopterTrajectoryGenerator;

int main(void) {
  // Simulation parameters
  int version = 4;
  int maxNumberObs = 1;
  int numTrajToGenerate = 1000000000;  //TODO try 100,000,000 on final run?

  // Input limits
  double fmin = 5;  //[m/s**2]
  double fmax = 30;  //[m/s**2]
  double wmax = 20;  //[rad/s]
  double minTimeSec = 0.002;  //[s]

  // Sampling ranges
  double posRange = 4.0;
  double velRange = 4.0;
  double accRange = 4.0;
  double timeRangeMin = 0.2;
  double timeRangeMax = 4.0;
  double obsSideMin = 0.1;
  double obsSideMax = 1.5;

  // Set up random number generator
  random_device rd;  //Will be used to obtain a seed for the random number engine
  mt19937 gen(rd());  //Standard mersenne_twister_engine seeded with rd()
  uniform_real_distribution<> randPos(-posRange, posRange);
  uniform_real_distribution<> randVel(-velRange, velRange);
  uniform_real_distribution<> randAcc(-accRange, accRange);
  uniform_real_distribution<> randTime(timeRangeMin, timeRangeMax);
  uniform_real_distribution<> randSideLen(obsSideMin, obsSideMax);

  // Constants
  Vec3d gravity = Vec3d(0, 0, -9.81);  //[m/s**2]
  Vec3d pos0(0, 0, 0);  //position

  ofstream monteSim, monteReadme;
  monteSim.open("Logs/monteSimV" + to_string(version) + ".csv");  // For storing data
  monteReadme.open("Logs/monteSimV" + to_string(version) + ".txt");  // For storing meta-parameters
  monteReadme << "maxNumberObs = " << maxNumberObs << endl;
  monteReadme << "numTrajToGenerate = " << numTrajToGenerate << endl;
  monteReadme << "fmin = " << fmin << endl;
  monteReadme << "fmax = " << fmax << endl;
  monteReadme << "wmax = " << wmax << endl;
  monteReadme << "minTimeSec = " << minTimeSec << endl;
  monteReadme << "posRange = " << posRange << endl;
  monteReadme << "velRange = " << velRange << endl;
  monteReadme << "accRange = " << accRange << endl;
  monteReadme << "timeRangeMin = " << timeRangeMin << endl;
  monteReadme << "timeRangeMax = " << timeRangeMax << endl;
  monteReadme << "obsSideMin = " << obsSideMin << endl;
  monteReadme << "obsSideMax = " << obsSideMax << endl;

  // Loop over number of obstacles
  for (int numObs = 1; numObs <= maxNumberObs; numObs *= 2) {
    cout << "Running simulations with " << numObs << " obstacles.\n";

    // Counter variables
    int numStateFeasible = 0;
    int numStateInfeasible = 0;
    int numStateIndeterminable = 0;

    long nsDefTime = 0;
    long nsGenTime = 0;
    long nsInputFeasTime = 0;
    long nsStateCheckTime = 0;
    long nsStateFeasCheck = 0;
    long nsStateInfeasCheck = 0;
    long nsStateIndetCheck = 0;

    high_resolution_clock::time_point startTime = high_resolution_clock::now();
    for (int simNum = 0; simNum < numTrajToGenerate; simNum++) {
      if (simNum % 100000 == 0) {
        cout << "\tFraction complete: "
             << double(simNum) / double(numTrajToGenerate) << endl;
      }

      high_resolution_clock::time_point startDef = high_resolution_clock::now();
      //Define the trajectory starting state:
      Vec3d vel0(randVel(gen), randVel(gen), randVel(gen));  //velocity
      Vec3d acc0(randAcc(gen), randAcc(gen), randAcc(gen));  //acceleration
      //define the goal state:
      Vec3d posf(randPos(gen), randPos(gen), randPos(gen));  //position
      Vec3d velf(randVel(gen), randVel(gen), randVel(gen));  //velocity
      Vec3d accf(randAcc(gen), randAcc(gen), randAcc(gen));  //acceleration
      //define the duration:
      double Tf = randTime(gen);

      RapidTrajectoryGenerator<double> traj(pos0, vel0, acc0, gravity);
      traj.SetGoalPosition(posf);
      traj.SetGoalVelocity(velf);
      traj.SetGoalAcceleration(accf);

      vector<shared_ptr<ConvexObj<double>>> obstacles;
      for (int i = 0; i < numObs; i++) {
        Vec3d objPos(randPos(gen), randPos(gen), randPos(gen));
        obstacles.push_back(
            make_shared<Sphere<double>>(objPos, randSideLen(gen)));
//        double sideLen = randSideLen(gen);
//        obstacles.push_back(
//                    make_shared<RectPrism<double>>(objPos, Vec3d(sideLen,sideLen,sideLen), Rotationd::Identity()));
      }
      high_resolution_clock::time_point endDef = high_resolution_clock::now();
      nsDefTime += duration_cast < nanoseconds > (endDef - startDef).count();

      // Generate trajectory
      high_resolution_clock::time_point startGen = high_resolution_clock::now();
      traj.Generate(Tf);
      high_resolution_clock::time_point endGen = high_resolution_clock::now();
      nsGenTime += duration_cast < nanoseconds > (endGen - startGen).count();

      // Check input feasibility
      high_resolution_clock::time_point startInputFeas =
          high_resolution_clock::now();
      RapidTrajectoryGenerator<double>::InputFeasibilityResult inputFeas = traj
          .CheckInputFeasibility(fmin, fmax, wmax, minTimeSec);
      high_resolution_clock::time_point endInputFeas =
          high_resolution_clock::now();
      if (inputFeas
          != RapidTrajectoryGenerator<double>::InputFeasibilityResult::InputFeasible) {
        // This trajectory is not input feasible, try again.
        simNum--;
        continue;
      }
      nsInputFeasTime += duration_cast < nanoseconds
          > (endInputFeas - startInputFeas).count();

      // Check state feasibility
      high_resolution_clock::time_point startStateFeas =
          high_resolution_clock::now();
      CollisionChecker<double>::CollisionResult stateFeas = CollisionChecker<
          double>::NoCollision;
      CollisionChecker<double> checker(traj.GetTrajectory());
      bool stateIndeterminableFound = false;
      for (auto obs : obstacles) {
        stateFeas = checker.CollisionCheck(obs, minTimeSec);
        if (stateFeas == CollisionChecker<double>::Collision) {
          stateIndeterminableFound = false;
          break;
        } else if (stateFeas
            == CollisionChecker<double>::CollisionIndeterminable) {
          stateIndeterminableFound = true;  //TODO: Should there be a break here? There would be in a realistic implementation
        }
      }
      high_resolution_clock::time_point endStateFeas =
          high_resolution_clock::now();
      long thisCheckTime = duration_cast < nanoseconds
          > (endStateFeas - startStateFeas).count();
      nsStateCheckTime += thisCheckTime;
      if (stateIndeterminableFound) {
        // At least on obstacle was indeterminable, but none were infeasible
        stateFeas = CollisionChecker<double>::CollisionIndeterminable;
      }

      switch (stateFeas) {
        case RapidTrajectoryGenerator<double>::StateFeasible:
          numStateFeasible++;
          nsStateFeasCheck += thisCheckTime;
          break;
        case RapidTrajectoryGenerator<double>::StateInfeasible:
          numStateInfeasible++;
          nsStateInfeasCheck += thisCheckTime;
          break;
        case RapidTrajectoryGenerator<double>::StateIndeterminable:
          numStateIndeterminable++;
          nsStateIndetCheck += thisCheckTime;
          break;
      }
    }
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    auto nsDuration = duration_cast < nanoseconds
        > (endTime - startTime).count();

    cout << endl;
    cout << "\tnumStateFeasible = " << numStateFeasible << endl;
    cout << "\tnumStateInfeasible = " << numStateInfeasible << endl;
    cout << "\tnumStateIndeterminable = " << numStateIndeterminable << endl
         << endl;

    cout << "\tTotal time [ns] = " << nsDuration << endl;
    cout << "\tusDefTime [ns] = " << nsDefTime << endl;
    cout << "\tusGenTime [ns] = " << nsGenTime << endl;
    cout << "\tusInputFeasTime [ns] = " << nsInputFeasTime << endl;
    cout << "\tusStateFeasTime [ns] = " << nsStateCheckTime << endl << endl;

    cout << "\tAverage def time [ns] = "
         << ((double) nsDefTime) / numTrajToGenerate << endl;
    cout << "\tAverage gen time [ns] = "
         << ((double) nsGenTime) / numTrajToGenerate << endl;
    cout << "\tAverage input feasibility check time [ns] = "
         << ((double) nsInputFeasTime) / numTrajToGenerate << endl;
    cout << "\tAverage state feasibility check time [ns] = "
         << ((double) nsStateCheckTime) / numTrajToGenerate << endl;
    cout << "\tAverage state feasibility check time per obstacle [ns] = "
         << ((double) nsStateCheckTime) / numTrajToGenerate / numObs << endl;
    cout
        << "\tAverage state feasibility check time per feasible obstacle [ns] = "
        << ((double) nsStateFeasCheck) / numStateFeasible / numObs << endl;
    cout
        << "\tAverage state feasibility check time per infeasible obstacle [ns] = "
        << ((double) nsStateInfeasCheck) / numStateInfeasible / numObs << endl;
    cout
        << "\tAverage state feasibility check time per indeterminable obstacle [ns] = "
        << ((double) nsStateIndetCheck) / numStateIndeterminable / numObs
        << endl << endl;

    cout << "\tPercent state feasible = "
         << 100.0 * ((double) numStateFeasible) / numTrajToGenerate << endl
         << endl;

    monteSim << numObs << ",";
    monteSim << numStateFeasible << ",";
    monteSim << numStateInfeasible << ",";
    monteSim << numStateIndeterminable << ",";
    monteSim << nsDuration << ",";
    monteSim << nsDefTime << ",";
    monteSim << nsGenTime << ",";
    monteSim << nsInputFeasTime << ",";
    monteSim << nsStateCheckTime << ",";
    monteSim << numTrajToGenerate << ",";
    monteSim << nsStateFeasCheck << ",";
    monteSim << nsStateInfeasCheck << ",";
    monteSim << nsStateIndetCheck << ",\n";
  }
  monteSim.close();
  monteReadme.close();

  return 0;
}

