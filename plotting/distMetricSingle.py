import transExternal
import readData
import json
import os
import numpy as np
import copy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# treat this as global

# create action list
numPts = 8
radius = 0.12
thDel = 2*np.pi/numPts

actions = [[radius*np.cos(i*thDel),radius*np.sin(i*thDel)]\
           for i in range(numPts)]

indexList = [0,6,3,7,4,2,6,4,1,5,2,0,4,2,7,3,0,6,2,0,5,1,6,4]
actionList = [actions[x] for x in indexList]

# end global

def distOs(o1,o2):
    return np.sqrt((o1[0]-o2[0])**2+(o1[1]-o2[1])**2)

def initialState(sF):
    sI = copy.deepcopy(sF)
    if sF[0] == 2:
        # revolute
        sI[2][0] = np.arctan2(-sI[1][1],-sI[1][0]) # atan2(-yp,-xp)
    elif sF[0] == 3:
        # prismatic
        c = np.cos(sF[1][2])
        s = np.sin(sF[1][2])
        if np.abs(c) >= np.abs(s):
            sI[2][0] = -sF[1][0]/c
        else:
            sI[2][0] = -sF[1][1]/s
    elif sF[0] == 4:
        # latch
        sI[2][0] = sI[1][3]
        sI[2][1] = sI[1][4]

    return sI

def calcDistMetric(r,m):
    if ((r[0] == 0) and (m[0] == 0)) or ((r[0] == 1) and (m[0] == 1)):
        return 0.0
    else:
        # get initial states
        rI = initialState(r)
        mI = initialState(m)
        
        dists = []
        for a in actionList:
            oR,rI = transExternal.simulate(rI,a)
            oM,mI = transExternal.simulate(mI,a)
            dists.append(distOs(oR,oM))
            
        return np.mean(dists)

## r = [4, [0.27, 0.0, 0.17, -3.14159, 0.1], [2.57172, 0.102363]]          
## m = [4, [0.27, 0.0, 0.17, -3.14159, 0.1], [1.57172, 0.102363]]          

## print calcDistMetric(r,m)

## r = [4, [0.27, 0.0, 0.17, -3.14159, 0.1], [2.57172, 0.102363]]          
## m = [4, [0.292522, 0.034128, 0.19, -3.02545, 0.104506], [2.71192, 0.0745014]]

## print calcDistMetric(r,m)

## m = [4, [0.322032, 0.069892, 0.19, -2.92787, 0.139529], [2.863, 0.0197544]]

## print calcDistMetric(r,m)


# pris
r = [3, [0.0, 0.226274, -1.5708], [0.340084]]

xs = np.linspace(-0.1,0.1,21)
ys = np.linspace(0.126274,0.326274,21)
ths = np.linspace(-np.pi,0.0,21)

#ys = np.linspace(-0.10,0.10,21)
#rads = np.linspace(0.07,0.27,21)
#ths = np.concatenate((np.linspace(-np.pi/2,-np.pi,10,endpoint=False),\
#                      np.linspace(np.pi/2,np.pi,11)))
#ds = np.linspace(0.01,0.21,21)

dists = []
for x in xs:
    m = [3, [x, 0.226274, -1.5708], [0.340084]]
    dists.append(calcDistMetric(r,m))

plt.plot(xs,dists,'-o')
plt.show()

dists = []
for y in ys:
    m = [3, [0.0, y, -1.5708], [0.340084]]
    dists.append(calcDistMetric(r,m))

plt.plot(ys,dists,'-o')
plt.show()

dists = []
for th in ths:
    m = [3, [0.0, 0.226274, th], [0.340084]]
    dists.append(calcDistMetric(r,m))

plt.plot(ths,dists,'-o')
plt.show()

# one 3d plot
dists = []
xs3d = []
ths3d = []
for x in xs:
    for th in ths:
        m = [3, [x, 0.226274, th], [0.340084]]
        dists.append(calcDistMetric(r,m))
        xs3d.append(x)
        ths3d.append(th)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(xs3d,ths3d,dists)
plt.show()

raise Exception

# latch
# the real state
r = [4, [0.27, 0.0, 0.17, -3.14159, 0.1], [2.57172, 0.102363]]

xs = np.linspace(0.17,0.37,21)
ys = np.linspace(-0.10,0.10,21)
rads = np.linspace(0.07,0.27,21)
ths = np.concatenate((np.linspace(-np.pi/2,-np.pi,10,endpoint=False),\
                      np.linspace(np.pi/2,np.pi,11)))
ds = np.linspace(0.01,0.21,21)

dists = []
for x in xs:
    m = [4, [x, 0.0, 0.17, -3.14159, 0.1], [2.57172, 0.102363]]
    dists.append(calcDistMetric(r,m))

plt.plot(xs,dists,'-o')
plt.show()
                             
dists = []
for y in ys:
    m = [4, [0.27, y, 0.17, -3.14159, 0.1], [2.57172, 0.102363]]
    dists.append(calcDistMetric(r,m))
    
plt.plot(ys,dists,'-o')
plt.show()            

dists = []
for rad in rads:
    m = [4, [0.27, 0.0, rad, -3.14159, 0.1], [2.57172, 0.102363]]
    dists.append(calcDistMetric(r,m))
    
plt.plot(rads,dists,'-o')
plt.show() 

dists = []
for th in ths :
    m = [4, [0.27, 0.0, 0.17, th, 0.1], [2.57172, 0.102363]]
    dists.append(calcDistMetric(r,m))
    
plt.plot(ths,dists,'-o')
plt.show()

dists = []
for d in ds:
    m = [4, [0.27, 0.0, 0.17, -3.14159, d], [2.57172, 0.102363]]
    dists.append(calcDistMetric(r,m))
    
plt.plot(ds,dists,'-o')
plt.show() 
