import transExternal
import readData
import json
import os
import numpy as np
import copy

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

#fbProbs, numSteps, model, statesInRbt, states, logProbs_T, logProbs_O, logProbs, poses, actions, obs, actionType, actionSelectionType, numMechanismTypes, numParticles, numRepeats, neff_fract, modelNums, realStates, BIAS, FTSD, FOSD, RTSD, ROSD = readData.getData(fileName)


# create action list
numPts = 8
radius = 0.12
thDel = 2*np.pi/numPts

actions = [[radius*np.cos(i*thDel),radius*np.sin(i*thDel)]\
           for i in range(numPts)]

indexList = [0,6,3,7,4,2,6,4,1,5,2,0,4,2,7,3,0,6,2,0,5,1,6,4]
actionList = [actions[x] for x in indexList]

# set up to read file
exeDir = os.path.abspath(os.path.dirname(__file__))
inFile = exeDir+'/rmStatesAll05_28_2015.txt'

# set up needed variables
relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)
numT = 50 # number of trials per setting

# set up holding lists
errors = [[] for y in range(numMT)]
misClass = [0]*numMT
misClassMT = [[0]*numMT for y in range(numMT)]

# get info from file
f = open(inFile,'r')
sSave = json.load(f)
rStatesSave = sSave['rStatesSave']
mStatesSave = sSave['mStatesSave']
f.close()

print len(rStatesSave)
print len(mStatesSave)


includeMisclass = False
typeString = ''
if includeMisclass:
    typeString = 'wM'
else:
    typeString = 'woM'

# set up to write file
outFile = exeDir+'/perfValsAll05_28_2015'+typeString+'.txt'


if includeMisclass:
    # iterate over states
    for rAll,mAll in zip(rStatesSave,mStatesSave):
        numS = len(rAll) # number of steps per trial
        errors[rAll[-1][0]].append([]) # add empty list to hold errors over a's

        # check for misclassification
        if rAll[-1][0] != mAll[-1][0]:
            # misclassified
            misClass[rAll[-1][0]] += 1
            misClassMT[rAll[-1][0]][mAll[-1][0]] += 1

        # and
        # calculate distance metric for every step
        for r,m in zip(rAll,mAll):
            if ((r[0] == 0) and (m[0] == 0)) or ((r[0] == 1) and (m[0] == 1)):
                errors[rAll[-1][0]][-1].append(0)
            else:
                # get initial states
                rI = initialState(r)
                mI = initialState(m)

                dists = []
                for a in actionList:
                    oR,rI = transExternal.simulate(rI,a)
                    oM,mI = transExternal.simulate(mI,a)
                    dists.append(distOs(oR,oM))

                errors[rAll[-1][0]][-1].append(np.mean(dists))

else:
    # iterate over states
    for rAll,mAll in zip(rStatesSave,mStatesSave):
        errors[rAll[-1][0]].append([]) # add empty list to hold errors over a's

        # check for misclassification
        if rAll[-1][0] != mAll[-1][0]:
            # misclassified
            misClass[rAll[-1][0]] += 1
            misClassMT[rAll[-1][0]][mAll[-1][0]] += 1
        else:
            # or
            # calculate distance metric for every step
            for r,m in zip(rAll,mAll):
                if ((r[0] == 0) and (m[0] == 0)) or \
                   ((r[0] == 1) and (m[0] == 1)):
                    errors[rAll[-1][0]][-1].append(0)
                else:
                    # get initial states
                    rI = initialState(r)
                    mI = initialState(m)

                    dists = []
                    for a in actionList:
                        oR,rI = transExternal.simulate(rI,a)
                        oM,mI = transExternal.simulate(mI,a)
                        dists.append(distOs(oR,oM))

                    errors[rAll[-1][0]][-1].append(np.mean(dists))

print "Errors:"
print errors
print "Misclassifcations:"
print misClass
print "Misclassifications per Model:"
print misClassMT

f2 = open(outFile,'w')
json.dump({'errors':errors,'misClass':misClass,'misClassMT':misClassMT},f2)
f2.close()

print "Average Error per Model:"
print [np.mean(np.vstack(es),0) for es in errors]
print "Standard Deviation per Model:"
print [np.std(np.vstack(es),0) for es in errors]
                             
            

