import json
import os
import numpy as np
import copy

# set date
date = "04_14_2015"
print date

# set up to read file
exeDir = os.path.abspath(os.path.dirname(__file__))
inFile = exeDir+'/rmStatesAll'+date+'.txt'

# set up needed variables
relevantMT = [0,1,2,3,4]
numMT = len(relevantMT)
numT = 50 # number of trials per setting

# get info from file
f = open(inFile,'r')
sSave = json.load(f)
rStatesSave = sSave['rStatesSave']
mStatesSave = sSave['mStatesSave']
f.close()

# get info from OG AS file
inFile2 = 'perfValsStateAll'+date+'.txt'
f2 = open(inFile2,'r')
sSave = json.load(f2)
errors2 = sSave['errors']
misClass2 = sSave['misClass']
misClassMT2 = sSave['misClassMT']
f2.close()

stateErrors = []
for es in errors2:
    stateErrors.extend(es)

print len(stateErrors)
count = 0
for r,m in zip(rStatesSave,mStatesSave):
    print r[-1]
    print m[-1]
    # the number of stateErrors per model type is different
    # gotta do funky stuff to make sure things come out the same
    if r[-1][0] == m[-1][0]:
        print stateErrors[count]
        count+=1
        
print len(errors2[2])
