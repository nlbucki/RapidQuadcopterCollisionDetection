# Rapid Collision Detection for Multicopter Trajectories

Nathan Bucki (nathan_bucki@berkeley.edu)

This contains source code implementing an algorithm for quickly detecting whether certain polynomial trajectories in time intersect with convex obstacles. The algorithm is used in conjunction with an existing multicopter trajectory generation method to achieve rapid, obstacle-aware motion planning in environments with both static convex obstacles and dynamic convex obstacles whose boundaries do not rotate.

The algorithm is described in a paper submitted to the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), and will be made available for download if accepted.

The existing multicopter trajectory generation method used in this project is described in the following paper: M.W. Mueller, M. Hehn, and R. D'Andrea, "A computationally efficient motion primitive for quadrocopter trajectory generation," IEEE Transactions on Robotics Volume 31, no.8, pages 1294-1310, 2015. The paper can be downloaded [here](https://hiperlab.berkeley.edu/publications/), and the original code can be downloaded [here](https://github.com/markwmuller/RapidQuadrocopterTrajectories).

## Getting Started

First clone the repository and entery the created folder:
```
git clone https://github.com/nlbucki/RapidQuadcopterCollisionDetection.git
cd RapidQuadcopterCollisionDetection
```

Create a build folder and compile:
```
mkdir build
cd build
cmake ..
make
```

Two programs are provided that both demonstrate the performance of the algorithm and give an example of how the algorithm can be used to generate collision free motion primitives. The programs `PerformanceEval` and `ForestPerformanceEval` perform the Monte Carlo simulation with random and constant obstacles respectively as described in Section IV of the associated paper. Both programs can be ran from the parent directory (after compiling the code) with the following commands:
```
./build/test/PerformanceEval
./build/test/ForestPerformanceEval
```

Both programs should output the average collision detection time required for trajectories that are feasible, infeasible, or of indeterminable feasibility. A python script is provided to visualize a single batch of candidate trajectories for the constant obstacle case, and can be ran using the command:
```
python ./scripts/plotSingleForestBatch.py
```

## Documentation
An HTML file generated with [Doxygen](http://www.doxygen.nl/) can be accesed after cloning the repository by opening `Documentation.html` in the `doc/` folder.

## Licensing

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see http://www.gnu.org/licenses/.