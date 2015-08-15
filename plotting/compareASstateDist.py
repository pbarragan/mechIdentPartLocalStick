import numpy as np
from matplotlib import pyplot
import json

# get info from Random AS file
inFile1 = 'perfValsStateAll04_14_2015.txt'
f1 = open(inFile1,'r')
sSave = json.load(f1)
errors1 = sSave['errors']
misClass1 = sSave['misClass']
misClassMT1 = sSave['misClassMT']
f1.close()

# get info from OG AS file
inFile2 = 'perfValsStateAll05_28_2015.txt'
f2 = open(inFile2,'r')
sSave = json.load(f2)
errors2 = sSave['errors']
misClass2 = sSave['misClass']
misClassMT2 = sSave['misClassMT']
f2.close()

# get info from OG AS file w/ bias
inFile3 = 'perfValsStateAll06_22_2015.txt'
f3 = open(inFile3,'r')
sSave = json.load(f3)
errors3 = sSave['errors']
misClass3 = sSave['misClass']
misClassMT3 = sSave['misClassMT']
f3.close()

# get info from OG AS file w/ bias
inFile4 = 'perfValsStateAll06_23_2015.txt'
f4 = open(inFile4,'r')
sSave = json.load(f4)
errors4 = sSave['errors']
misClass4 = sSave['misClass']
misClassMT4 = sSave['misClassMT']
f4.close()

print "Random Error Stats"
for i in range(len(errors1)):
    print str(i)+':',np.mean(np.array(errors1[i])),',',\
        np.std(np.array(errors1[i]))

print "Racing Error Stats"
for i in range(len(errors2)):
    print str(i)+':',np.mean(np.array(errors2[i])),',',\
        np.std(np.array(errors2[i]))

print "Random Error Stats w/ Bias"
for i in range(len(errors3)):
    print str(i)+':',np.mean(np.array(errors3[i])),',',\
        np.std(np.array(errors3[i]))

print "Racing Error Stats w/ Bias"
for i in range(len(errors4)):
    print str(i)+':',np.mean(np.array(errors4[i])),',',\
        np.std(np.array(errors4[i]))
