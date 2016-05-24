import transExternal
import readData
import json
import os
import sys
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

# get the relevant name info
dateName = sys.argv[1].rsplit(".",1)[0].rsplit("fbProbsAll",1)[1]

# set up to read file
exeDir = os.path.abspath(os.path.dirname(__file__))
inFile = exeDir+'/'+sys.argv[1]

# set up needed variables
relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)
numT = 50 # number of trials per setting
numSteps = 11 # number of steps per trial

# set up holding lists
errors = [[] for y in range(numMT)]
misClass = [0]*numMT
misClassMT = [[0]*numMT for y in range(numMT)]

# get info from file
f = open(inFile,'r')
sSave = json.load(f)
fbProbsSave = sSave['fbProbsSave']
f.close()

#print fbProbsSave
print len(fbProbsSave)

# set up to write file
outFile = exeDir+'/avgProbsAll'+dateName+'.txt'

# list of model types
#    list of steps
#       list of probs per model at a given step



print "Avgs:"    
for fbProbsList in fbProbsSave:
    ps = [list([]) for _ in xrange(numSteps)]
    avgs = []
    for fbProbs in fbProbsList:
        print len(fbProbs)
        for i,stepProbs in enumerate(fbProbs):
            ps[i].append(stepProbs)

    for p in ps:
        avgs.append(np.mean(np.vstack(p),0))

    print avgs    

    
probAvgs = [np.mean(np.vstack(fbProbsPerModel),0) for fbProbsPerModel in fbProbsList]
print probAvgs
print "Std Devs:"
probStdDevs = [np.std(np.vstack(fbProbsPerModel),0) for fbProbsPerModel in fbProbsSave]
print probStdDevs

f2 = open(outFile,'w')
json.dump({'probAvgs':probAvgs,'probStdDevs':probStdDevs},f2)
f2.close()
