# Rapid Collision Detection for Multicopter Trajectories

Nathan Bucki (nathan_bucki@berkeley.edu)

This contains source code implementing an algorithm for quickly detecting whether certain polynomial trajectories in time intersect with convex obstacles. The algorithm is used in conjunction with an existing multicopter trajectory generation method to achieve rapid, obstacle-aware motion planning in environments with both static convex obstacles and dynamic convex obstacles whose boundaries do not rotate.

The algorithm is described in a paper submitted to the IEEE/RSJ International Conference on Intellient Robots and Systems (IROS), and will be made available for download if accepted.

The existing multicopter trajectory generation method used in this project is described in the following paper: M.W. Mueller, M. Hehn, and R. D'Andrea, "A computationally efficient motion primitive for quadrocopter trajectory generation," IEEE Transactions on Robotics Volume 31, no.8, pages 1294-1310, 2015. The paper can be downloaded [here](https://hiperlab.berkeley.edu/publications/), and the original code can be downloaded [here](https://github.com/markwmuller/RapidQuadrocopterTrajectories).

## Licensing

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see http://www.gnu.org/licenses/.