import json
import os
import numpy as np
import copy

# set up to read file
exeDir = os.path.abspath(os.path.dirname(__file__))
inFile = exeDir+'/rmStatesAll05_28_2015.txt'

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


for r,m in zip(rStatesSave,mStatesSave):
    print r[-1]
    print m[-1]
