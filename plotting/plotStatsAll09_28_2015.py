# this is for the 6/23/2015 experiments
# 50 trials per
# 80% bias level, racing action selection

import readData
import heapq
import math
import numpy as np

import statsFiles09_28_2015
import json
import os

import auxUtils

from matplotlib import pyplot

exeDir = os.path.abspath(os.path.dirname(__file__))
print exeDir
dirName = '/home/barragan/dataPostGrad/2015_09_28/'

outFile = exeDir+'/rmStatesAll09_28_2015.txt'

numET = 1 # no parameter variations
relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)
numT = 50 # number of trials per setting
#numS = 10 # number of actions (steps) per experiment

#errors = [[[] for y in range(numMT)] for x in range(numET)]
#misClass = [[0]*numMT for x in range(numET)]
#misClassMT = [[[0]*numMT for y in range(numMT)] for x in range(numET)]

# goes experiment type (0-3), model type (0-3), trial number (0-9)
if(True):
    files = statsFiles09_28_2015.files
    # nothing to change below here
    fileNames = [[[] for y in range(numMT)] for x in range(numET)]


    for i in range(numET):
        for j in range(numMT):
            fileNames[i][j] = files[numMT*numT*i+numT*j:numMT*numT*i+numT*j+numT]


    rStatesSave = []
    mStatesSave = []

    for i,fileLists in enumerate(fileNames):
        for j,fList in enumerate(fileLists):
            dists = []

            for f in fList:
                print i,j
                print f

                fbProbs, numSteps, model, statesInRbt, states, logProbs_T, logProbs_O, logProbs, poses, actions, obs, \
                         actionType, actionSelectionType, numMechanismTypes, numParticles, numRepeats, neff_fract, \
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
                    #errors[i][j].append(auxUtils.stateDist2(realStates[-1],\
                    #                                        maxState))

    f = open(outFile,'w')
    json.dump({'rStatesSave':rStatesSave,'mStatesSave':mStatesSave},f)
    f.close()
else:
    f = open(outFile,'r')
    sSave = json.load(f)
    rStatesSave = sSave['rStatesSave']
    mStatesSave = sSave['mStatesSave']
    f.close()

    print len(rStatesSave)
    print len(mStatesSave)

    for i in range(numET):
        for j in range(numMT):
            for k in range(numT):
                sInd = numMT*numT*i+numT*j+k
                e=auxUtils.stateDist2(rStatesSave[sInd],
                                      mStatesSave[sInd])
                if e<1000:
                    errors[i][j].append(e)
                else:
                    misClass[i][j]+=1
                    misClassMT[i][j][relevantMT.index(mStatesSave[sInd][0])]+=1
                    print "This is a huge ERROR!"
                #print rStatesSave[sInd]
                #print mStatesSave[sInd]
                #print auxUtils.stateDist2(rStatesSave[sInd],
                #                          mStatesSave[sInd])

raise Exception
print errors


typeAndModelError = [[] for x in range(numET)]
typeError = []
modelError = []

typeAndModelErrorSD = [[] for x in range(numET)]
typeErrorSD = []
modelErrorSD = []

# mean
for i,typeEs in enumerate(errors):
    for modelEs in typeEs:
        typeAndModelError[i].append(np.mean(modelEs))
        typeAndModelErrorSD[i].append(np.std(modelEs))

hold = [[] for x in range(numMT)]

for es in typeAndModelError:
    typeError.append(np.mean(es))
    typeErrorSD.append(np.std(es))    
    for i,e in enumerate(es):
        hold[i].append(e)

for es in hold:
    modelError.append(np.mean(es))
    modelErrorSD.append(np.std(es))

print
print
print
print "[Type][Model]"
print "RMSE:",typeAndModelError
print "SD:",typeAndModelErrorSD
print "[Type]"
print "RMSE:",typeError
print "SD:",typeErrorSD
print "[Model]"
print "RMSE:",modelError
print "SD:",modelErrorSD


models = ["Free","Fixed","Rev","Pris","Latch"]
types = ["10000"]
typesF = [float(x) for x in types]

print
auxUtils.printTable(models,types,typeAndModelError,typeAndModelErrorSD)
print
auxUtils.printFlatTable(types,typeError,typeErrorSD)
print
auxUtils.printFlatTable(models,modelError,modelErrorSD)
print
#auxUtils.printTableNoSD(models,types,misClass)        
print "Misclassification Error:"
auxUtils.printMisClassTable(models,types,misClass,misClassMT)
if(False):

    pyplot.scatter(typesF,typeError)
    pyplot.errorbar(typesF,typeError,yerr=typeErrorSD)
    pyplot.xlabel('Number of Particles')
    pyplot.ylabel('Error [m]')
    
    pyplot.show()
