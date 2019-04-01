from __future__ import print_function, division

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.lines import Line2D


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
fname = './data/ForestPerfEval.csv'
fname2 = './data/ForestPerfEvaltreeParamsCSV.csv'
trajData = np.genfromtxt(fname, delimiter=',')
treeData = np.genfromtxt(fname2, delimiter=',')

# Set up plot
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_zlim(-2.5, 2.5)
ax.view_init(20, -140)

# Plot trees
treePos = treeData[:, 0:3]
treeSize = treeData[:, 3:6]
treeRot = treeData[:, 6:15]
for pos, size, rot in zip(treePos, treeSize, treeRot):
    obstacle = np.array([[-size[0] / 2.0, -size[1] / 2.0, -size[2] / 2.0],
                        [size[0] / 2.0, -size[1] / 2.0, -size[2] / 2.0],
                        [size[0] / 2.0, size[1] / 2.0, -size[2] / 2.0],
                        [-size[0] / 2.0, size[1] / 2.0, -size[2] / 2.0],
                        [-size[0] / 2.0, -size[1] / 2.0, size[2] / 2.0],
                        [size[0] / 2.0, -size[1] / 2.0, size[2] / 2.0],
                        [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0],
                        [-size[0] / 2.0, size[1] / 2.0, size[2] / 2.0]])
    prism = Poly3DCollection([], linewidths=1, edgecolors='k', facecolors='b', alpha=.5 , linewidth=0.5)
    ax.add_collection3d(prism)
    for i, point in enumerate(obstacle):
        R = np.array(rot).reshape((3,3))
        obstacle[i, :] = R.dot(point) + pos
    verts = getRectPrismPolygons(obstacle)
    prism.set_verts(np.multiply(verts, 1))

# For calculations
totalGen = 0
totalInputCheck = 0
totalStateCheck = 0

# Plot trajectoryies
for k in range(trajData.shape[0]):
    c0 = trajData[k, 0:3]
    c1 = trajData[k, 3:6]
    c2 = trajData[k, 6:9]
    c3 = trajData[k, 9:12]
    c4 = trajData[k, 12:15]
    c5 = trajData[k, 15:18]
    Tf = trajData[k, 18]
    inputFeas = trajData[k, 19]
    stateFeas = trajData[k, 20]
    nsGenTime = trajData[k, 21]
    nsInputFeasTime = trajData[k, 22]
    nsStateCheckTime = trajData[k, 23]
    
    totalGen += nsGenTime
    totalInputCheck += nsInputFeasTime
    totalStateCheck += nsStateCheckTime
    
    numPlotPoints = 100
    t = np.linspace(0, Tf, numPlotPoints)
    position = np.zeros([numPlotPoints, 3])
    for i in range(numPlotPoints):
        position[i, :] = c0 * t[i] ** 5 + c1 * t[i] ** 4 + c2 * t[i] ** 3 + c3 * t[i] ** 2 + c4 * t[i] + c5
    
    # Only plot trajectories that are input feasible
    if inputFeas == 0:  # Input is feasible
        if stateFeas == 0:  # State feasible
            ax.plot(position[:, 0], position[:, 1], position[:, 2], 'g', linewidth=0.5)
        else:  # State infeasible or indeterminable
            ax.plot(position[:, 0], position[:, 1], position[:, 2], 'r:', linewidth=0.5)
 
custom_lines = [Line2D([0], [0], color='g'),
                Line2D([0], [0], color='r', linestyle=':')]

ax.legend(custom_lines, ['No Collision', 'Collision'])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z', labelpad=-1.9)
plt.tight_layout(pad=0.01)

print('Total gen time [us] = ', float(totalGen) / 1000)
print('Total input check time [us] = ', float(totalInputCheck) / 1000)
print('Total state check time [us] = ', float(totalStateCheck) / 1000)
 
plt.show()
