import readData
import sys

f = sys.argv[1] 

fbProbs, numSteps, model, statesInRbt, states, logProbs_T, \
         logProbs_O, logProbs, poses, actions, obs, \
         actionType, actionSelectionType, numMechanismTypes, \
         numParticles, numRepeats, neff_fract, \
         modelNums, realStates, BIAS, FTSD, FOSD, RTSD, ROSD \
         = readData.get_data(f)

print fbProbs
