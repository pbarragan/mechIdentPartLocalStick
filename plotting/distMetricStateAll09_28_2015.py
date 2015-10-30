import numpy as np
from matplotlib import pyplot
import json
import os


from distMetricState import distState as dS

# set up needed variables
relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)

# set up holding lists
errors = [[] for y in range(numMT)]
misClass = [0]*numMT
misClassMT = [[0]*numMT for y in range(numMT)]

# set up to read file
exeDir = os.path.abspath(os.path.dirname(__file__))
inFile = exeDir+'/rmStatesAll09_28_2015.txt'

# set up to write file
outFile = exeDir+'/perfValsStateAll09_28_2015.txt'

# get info from file 
f = open(inFile,'r')
sSave = json.load(f)
rStatesSave = sSave['rStatesSave']
mStatesSave = sSave['mStatesSave']
f.close()

print len(rStatesSave)
print len(mStatesSave)

for i,(rAll,mAll) in enumerate(zip(rStatesSave,mStatesSave)):
    #print i
    # check for misclassification
    if rAll[-1][0] != mAll[-1][0]:
        # misclassified
        misClass[rAll[-1][0]] += 1
        misClassMT[rAll[-1][0]][mAll[-1][0]] += 1
    else:
        # or
        # calculate state distance metric for last step
        errors[rAll[-1][0]].append([]) # add empty list to hold errors over a's
        errors[rAll[-1][0]][-1].append(dS(rAll[-1],mAll[-1]))
        '''
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
        '''

f2 = open(outFile,'w')
json.dump({'errors':errors,'misClass':misClass,'misClassMT':misClassMT},f2)
f2.close()

print "Errors:"
print errors
print "Misclassifcations:"
print misClass
print "Misclassifications per Model:"
print misClassMT

print "Error Stats"
for i in range(len(errors)):
    print str(i)+':',np.mean(np.array(errors[i]))
