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

# get info from RandomB AS file
inFile1B = 'perfValsAll12_21_2015_randomwM.txt'
f1B = open(inFile1B,'r')
sSave = json.load(f1B)
errors1B = sSave['errors']
misClass1B = sSave['misClass']
misClassMT1B = sSave['misClassMT']
f1B.close()

# get info from OG AS file
inFile2 = 'perfValsAll05_28_2015wM.txt'
f2 = open(inFile2,'r')
sSave = json.load(f2)
errors2 = sSave['errors']
misClass2 = sSave['misClass']
misClassMT2 = sSave['misClassMT']
f2.close()

# get info from OG AS file
inFile3 = 'perfValsAll12_21_2015_simplewM.txt'
f3 = open(inFile3,'r')
sSave = json.load(f3)
errors3 = sSave['errors']
misClass3 = sSave['misClass']
misClassMT3 = sSave['misClassMT']
f3.close()

print 'Misclassifications Random'
print misClass1
print 'Misclassifications OG'
print misClass2
print 'Misclassifications Simple'
print misClass3

# calculate averages
print "Average Error per Model Random:"
means1 = [np.mean(np.vstack(es),0) for es in errors1]
print means1
print "Standard Deviation per Model Random:"
stds1 = [np.std(np.vstack(es),0) for es in errors1]
print stds1

print "Average Error per Model RandomB:"
means1B = [np.mean(np.vstack(es),0) for es in errors1B]
print means1B
print "Standard Deviation per Model RandomB:"
stds1B = [np.std(np.vstack(es),0) for es in errors1B]
print stds1B

print "Average Error per Model OG:"
means2 = [np.mean(np.vstack(es),0) for es in errors2]
print means2
print "Standard Deviation per Model OG:"
stds2 = [np.std(np.vstack(es),0) for es in errors2]
print stds2

print "Average Error per Model Simple:"
means3 = [np.mean(np.vstack(es),0) for es in errors3]
print means3
print "Standard Deviation per Model OG:"
stds3 = [np.std(np.vstack(es),0) for es in errors3]
print stds3

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

# get info from simple AS file
inFile7 = 'perfValsAll09_28_2015wM.txt'
f7 = open(inFile7,'r')
sSave = json.load(f7)
errors7 = sSave['errors']
misClass7 = sSave['misClass']
misClassMT7 = sSave['misClassMT']
f7.close()

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

print "Average Error per Model Simple:"
means7 = [np.mean(np.vstack(es),0) for es in errors7]
print means7
print "Standard Deviation per Model Simple:"
stds7 = [np.std(np.vstack(es),0) for es in errors7]
print stds7

# GET BEST
inFile6 = 'perfValsBest06_18_2015wM.txt'
f6 = open(inFile6,'r')
sSave = json.load(f6)
errors6 = sSave['errors']
misClass6 = sSave['misClass']
misClassMT6 = sSave['misClassMT']
f6.close()

print errors6

# plot
# steps of the experiments
steps = np.arange(1,11)
models = ["Free","Fixed","Rev","Pris","Latch"]
for ms1,sds1,ms1B,sds1B,ms2,sds2,ms3,sds3,ms4,sds4,ms5,sds5,ms7,sds7,esBest,mName in \
    zip(means1,stds1,means1B,stds1B,means2,stds2,means3,stds3,\
        means4,stds4,means5,stds5,means7,stds7,errors6,models):

    # no bias
    # plot random
    pyplot.errorbar(steps,ms1,yerr=sds1,fmt='-o',color='b')
    pyplot.plot(steps,ms1,color='b',marker='o',lw=2,markersize=8,\
                markeredgecolor='b',label='Random')
    # plot random
    pyplot.errorbar(steps,ms1B,yerr=sds1B,fmt='--o',color='b')
    pyplot.plot(steps,ms1B,color='b',marker='o',lw=2,markersize=8,\
                markeredgecolor='b',label='RandomB')
    # plot racing
    pyplot.errorbar(steps,ms2,yerr=sds2,fmt='-o',color='r')
    pyplot.plot(steps,ms2,color='r',marker='o',lw=2,markersize=8,\
                markeredgecolor='r',label='Racing')
    # plot simple
    pyplot.errorbar(steps,ms3,yerr=sds3,fmt='-o',color='k')
    pyplot.plot(steps,ms3,color='k',marker='o',lw=2,markersize=8,\
                markeredgecolor='k',label='Simple')
    

    # 80% bias level
    # plot random
    pyplot.errorbar(steps,ms4,yerr=sds4,fmt='-o',color='c')
    pyplot.plot(steps,ms4,color='c',marker='o',lw=2,markersize=8,\
                markeredgecolor='c',label='Random w/ Bias')
    # plot racing
    pyplot.errorbar(steps,ms5,yerr=sds5,fmt='-o',color='y')
    pyplot.plot(steps,ms5,color='y',marker='o',lw=2,markersize=8,\
                markeredgecolor='y',label='Racing w/ Bias')

    # plot simple
    pyplot.errorbar(steps,ms7,yerr=sds7,fmt='-o',color='m')
    pyplot.plot(steps,ms7,color='m',marker='o',lw=2,markersize=8,\
                markeredgecolor='m',label='Simple w/ Bias')

    # plot best
    #pyplot.errorbar(steps,ms2,yerr=sds2,fmt='-o',color='g')
    pyplot.plot(steps,esBest[0],color='g',marker='o',lw=2,markersize=8,\
                markeredgecolor='g',label='Best')
    
    pyplot.legend()
    pyplot.xlabel('Steps')
    pyplot.ylabel('Error [m]')

    #outFile = '/mit/barragan/Public/LIS/postGradFigures/compareAS'+mName+'.pdf'
    #outFile = 'dataPlots/handChosen/compareAS'+mName+'.pdf'
    #pyplot.savefig(outFile,bbox_inches='tight')
    pyplot.show()
    #pyplot.close()


# old latch best
#[[0.12010917110791212, 0.1459857869175755, 0.2140532579871118, 0.2166644391965765, 0.05540713029693328, 0.10987775169556734, 0.07085534126292667, 0.019242210533691684, 0.02724782002505467, 0.15543671332008385]]]

# new latch best
#[[0.2695814617736105, 0.10408954881434607, 0.10601747403123568, 0.11979776536033553, 0.041011480292654945, 0.09884274553810786, 0.11418362321694721, 0.0572749162071304, 0.12457289457161684, 0.09344081203818737]]]
