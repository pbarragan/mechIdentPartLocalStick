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
stds2 = [np.std(np.vstack(es),0) for es in errors1]
print stds2

# plot
# steps of the experiments
steps = np.arange(1,11)
models = ["Free","Fixed","Rev","Pris","Latch"]
for ms1,sds1,ms2,sds2,mName in zip(means1,stds1,means2,stds2,models):
    pyplot.errorbar(steps,ms1,yerr=sds1,fmt='-o',color='b')
    pyplot.plot(steps,ms1,color='b',marker='o',lw=2,markersize=8,\
                markeredgecolor='b',label='Random')
    pyplot.errorbar(steps,ms2,yerr=sds2,fmt='-o',color='r')
    pyplot.plot(steps,ms2,color='r',marker='o',lw=2,markersize=8,\
                markeredgecolor='b',label='Racing')
    pyplot.legend()
    pyplot.xlabel('Steps')
    pyplot.ylabel('Error [m]')

    outFile = '/mit/barragan/Public/LIS/postGradFigures/compareAS'+mName+'.pdf'
    pyplot.savefig(outFile,bbox_inches='tight')
    pyplot.show()
