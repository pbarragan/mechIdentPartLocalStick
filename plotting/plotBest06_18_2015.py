# this is for the 5/28/2015 experiments
# 50 trials per

# 1) plotBest06_18_2015.py
# 2) distMetricBest06_18_2015.py
# 3) compareAs.py

import readData
import heapq
import math
import numpy as np

import statsFiles05_28_2015
import json
import os

import auxUtils

from matplotlib import pyplot

exeDir = os.path.abspath(os.path.dirname(__file__))
print exeDir
#dirName = '/home/barragan/dataPostGrad/2015_05_28/'
dirName = '../'
outFile = exeDir+'/rmStatesBest06_18_2015.txt'

relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)

rStatesSave = []
mStatesSave = []

fList = ['data/2015_06_19/data0Fri_Jun_19_00_13_14_2015.txt','data/2015_06_21/data1Sun_Jun_21_17_20_14_2015.txt','data/2015_06_21/data2Sun_Jun_21_17_37_49_2015.txt','data/2015_06_21/data3Sun_Jun_21_21_04_04_2015.txt','data/2015_06_21/data4Sun_Jun_21_21_13_36_2015.txt']

for f in fList:
    print f

    fbProbs, numSteps, model, statesInRbt, states, logProbs_T, logProbs_O, \
        logProbs, poses, actions, obs, \
        actionType, actionSelectionType, numMechanismTypes, numParticles, \
        numRepeats, neff_fract, \
        modelNums, realStates, BIAS, FTSD, FOSD, RTSD, ROSD \
        = readData.get_data(dirName+f)

    # save real
    rStatesSave.append(realStates[1:])

    # find max state at each step
    mStatesSave.append([])
    for k,(Ss,lPs) in enumerate(zip(states[1:],logProbs[1:])):
        maxProbs = []
        maxProbInds = []
        for l in relevantMT:
            maxProbs.append(max(lPs[l]))
            maxProbInds.append(lPs[l].index(maxProbs[-1]))

        maxModelInd = maxProbs.index(max(maxProbs))
        maxProbInd = maxProbInds[maxModelInd]
        maxState = Ss[relevantMT[maxModelInd]][maxProbInd]

        print 'Real State:',realStates[k+1]
        print 'Max State:',maxState
        mStatesSave[-1].append(maxState)

f = open(outFile,'w')
json.dump({'rStatesSave':rStatesSave,'mStatesSave':mStatesSave},f)
f.close()
