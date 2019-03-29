from __future__ import print_function, division

import sys
import numpy as np
import matplotlib.pyplot as plt
from py3dmath import Rotation, Vec3
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import quadrocoptertrajectory as quadtraj
from matplotlib.lines import Line2D
from ModifyPlotForPublication import *

# Returns polygons from list of points of rectangular prism
def getRectPrismPolygons(Z):
    polys = [[Z[0], Z[1], Z[2], Z[3]],
             [Z[4], Z[5], Z[6], Z[7]],
             [Z[0], Z[1], Z[5], Z[4]],
             [Z[2], Z[3], Z[7], Z[6]],
             [Z[1], Z[2], Z[6], Z[5]],
             [Z[4], Z[7], Z[3], Z[0]]]
    return polys

# read the data:
fname = '../data/forestSimV4.csv'
fname2 = '../data/forestTreeParamsV4.csv'
trajData = np.genfromtxt(fname, delimiter=',')
treeData = np.genfromtxt(fname2, delimiter=',')

# Set up plot
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.56)
ax.set_zlim(-2.5, 2.5)
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
ax.view_init(20, -140)

# Plot trees
treePos = treeData[:,0:3]
treeSize = treeData[:,3:6]
treeRot = treeData[:,6:9]
for pos, size, rot in zip(treePos, treeSize, treeRot):
    obstacle = np.array([[-size[0]/2.0, -size[1]/2.0, -size[2]/2.0],
                        [size[0]/2.0, -size[1]/2.0, -size[2]/2.0],
                        [size[0]/2.0, size[1]/2.0, -size[2]/2.0],
                        [-size[0]/2.0, size[1]/2.0, -size[2]/2.0],
                        [-size[0]/2.0, -size[1]/2.0, size[2]/2.0],
                        [size[0]/2.0, -size[1]/2.0, size[2]/2.0],
                        [size[0]/2.0, size[1]/2.0, size[2]/2.0],
                        [-size[0]/2.0, size[1]/2.0, size[2]/2.0]])
    prism = Poly3DCollection([], linewidths=1, edgecolors='k', facecolors='b', alpha=.5 ,linewidth=0.5)
    ax.add_collection3d(prism)
    for i, point in enumerate(obstacle):
        R = Rotation.from_vector_part_of_quaternion(rot)
        obstacle[i, :] = Vec3.to_array(R*Vec3(point)).T[0, :] + pos
    verts = getRectPrismPolygons(obstacle)
    prism.set_verts(np.multiply(verts, 1))

# For calculations
totalGen = 0
totalInputCheck = 0
totalStateCheck = 0

# Plot trajectoryies
for k in range(trajData.shape[0]):
    pos0 = trajData[k,0:3]
    vel0 = trajData[k,3:6]
    acc0 = trajData[k,6:9]
    posf = trajData[k,9:12]
    velf = trajData[k,12:15]
    accf = trajData[k,15:18]
    Tf = trajData[k,18]
    inputFeas = trajData[k,19]
    stateFeas = trajData[k,20]
    nsDuration = trajData[k,21]
    nsGenTime = trajData[k,22]
    nsInputFeasTime = trajData[k,23]
    nsStateCheckTime = trajData[k,24]
    
    totalInputCheck += nsInputFeasTime
    totalStateCheck += nsStateCheckTime
    
    gravity = Vec3(0,0,-9.81)
    traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)
    traj.generate(Tf)
    
    numPlotPoints = 100
    time = np.linspace(0, Tf, numPlotPoints)
    position = np.zeros([numPlotPoints, 3])
    for i in range(numPlotPoints):
        t = time[i]
        position[i, :] = traj.get_position(t)
    
    if inputFeas == 0: # Input is feasible
        if stateFeas == 0: # State feasible
            ax.plot(position[:,0], position[:,1], position[:,2], 'g', linewidth=0.5)
        else:# State infeasible or indeterminable
            ax.plot(position[:,0], position[:,1], position[:,2], 'r:', linewidth=0.5)
 
custom_lines = [Line2D([0], [0], color='g'),
                Line2D([0], [0], color='r',linestyle=':')]

ax.legend(custom_lines, ['No Collision', 'Collision'])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z',labelpad=-1.9)
plt.tight_layout(pad=0.01)
fig.savefig('forestSim.pdf', bbox_inches='tight')

print('Total gen time [us] = ', float(totalGen)/1000)
print('Total input check time [us] = ', float(totalInputCheck)/1000)
print('Total state check time [us] = ', float(totalStateCheck)/1000)
 
plt.show()