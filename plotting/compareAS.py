# 1) plotBest06_18_2015.py
# 2) distMetricBest06_18_2015.py
# 3) compareAs.py

import numpy as np
from matplotlib import pyplot
import json

# get info from Random AS file
inFile1 = 'perfValsAll04_14_2015wM.txt'
f1 = open(inFile1,'r')
sSave = json.load(f1)
errors1 = sSave['errors']
misClass1 = sSave['misClass']
misClassMT1 = sSave['misClassMT']
f1.close()

# get info from OG AS file
inFile2 = 'perfValsAll05_28_2015wM.txt'
f2 = open(inFile2,'r')
sSave = json.load(f2)
errors2 = sSave['errors']
misClass2 = sSave['misClass']
misClassMT2 = sSave['misClassMT']
f2.close()

print 'Misclassifications Random'
print misClass1
print 'Misclassifications OG'
print misClass2

# calculate averages
print "Average Error per Model Random:"
means1 = [np.mean(np.vstack(es),0) for es in errors1]
print means1
print "Standard Deviation per Model Random:"
stds1 = [np.std(np.vstack(es),0) for es in errors1]
print stds1

print "Average Error per Model OG:"
means2 = [np.mean(np.vstack(es),0) for es in errors2]
print means2
print "Standard Deviation per Model OG:"
stds2 = [np.std(np.vstack(es),0) for es in errors2]
print stds2

# with 80% bias added

# get info from Random AS file
inFile4 = 'perfValsAll06_22_2015wM.txt'
f4 = open(inFile4,'r')
sSave = json.load(f4)
errors4 = sSave['errors']
misClass4 = sSave['misClass']
misClassMT4 = sSave['misClassMT']
f4.close()

# get info from OG AS file
inFile5 = 'perfValsAll06_23_2015wM.txt'
f5 = open(inFile5,'r')
sSave = json.load(f5)
errors5 = sSave['errors']
misClass5 = sSave['misClass']
misClassMT5 = sSave['misClassMT']
f5.close()

print 'Misclassifications Random'
print misClass4
print 'Misclassifications OG'
print misClass5

# calculate averages
print "Average Error per Model Random:"
means4 = [np.mean(np.vstack(es),0) for es in errors4]
print means4
print "Standard Deviation per Model Random:"
stds4 = [np.std(np.vstack(es),0) for es in errors4]
print stds4

print "Average Error per Model OG:"
means5 = [np.mean(np.vstack(es),0) for es in errors5]
print means5
print "Standard Deviation per Model OG:"
stds5 = [np.std(np.vstack(es),0) for es in errors5]
print stds5

# GET BEST
inFile3 = 'perfValsBest06_18_2015wM.txt'
f3 = open(inFile3,'r')
sSave = json.load(f3)
errors3 = sSave['errors']
misClass3 = sSave['misClass']
misClassMT3 = sSave['misClassMT']
f3.close()

print errors3

# plot
# steps of the experiments
steps = np.arange(1,11)
models = ["Free","Fixed","Rev","Pris","Latch"]
for ms1,sds1,ms2,sds2,ms4,sds4,ms5,sds5,esBest,mName in \
    zip(means1,stds1,means2,stds2,means4,stds4,means5,stds5,errors3,models):

    # no bias
    # plot random
    pyplot.errorbar(steps,ms1,yerr=sds1,fmt='-o',color='b')
    pyplot.plot(steps,ms1,color='b',marker='o',lw=2,markersize=8,\
                markeredgecolor='b',label='Random')
    # plot racing
    pyplot.errorbar(steps,ms2,yerr=sds2,fmt='-o',color='r')
    pyplot.plot(steps,ms2,color='r',marker='o',lw=2,markersize=8,\
                markeredgecolor='r',label='Racing')

    # 80% bias level
    # plot random
    pyplot.errorbar(steps,ms4,yerr=sds4,fmt='-o',color='c')
    pyplot.plot(steps,ms4,color='c',marker='o',lw=2,markersize=8,\
                markeredgecolor='c',label='Random w/ Bias')
    # plot racing
    pyplot.errorbar(steps,ms5,yerr=sds5,fmt='-o',color='y')
    pyplot.plot(steps,ms5,color='y',marker='o',lw=2,markersize=8,\
                markeredgecolor='y',label='Racing w/ Bias')

    # plot best
    #pyplot.errorbar(steps,ms2,yerr=sds2,fmt='-o',color='g')
    pyplot.plot(steps,esBest[0],color='g',marker='o',lw=2,markersize=8,\
                markeredgecolor='g',label='Best')
    
    pyplot.legend()
    pyplot.xlabel('Steps')
    pyplot.ylabel('Error [m]')

    outFile = '/mit/barragan/Public/LIS/postGradFigures/compareAS'+mName+'.pdf'
    #outFile = 'dataPlots/handChosen/compareAS'+mName+'.pdf'
    pyplot.savefig(outFile,bbox_inches='tight')
    pyplot.show()
    #pyplot.close()
