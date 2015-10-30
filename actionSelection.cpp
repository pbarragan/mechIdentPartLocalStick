#include "globalVars.h"
#include "timing.h"

#include <algorithm>
#include <numeric>
#include <iostream> // cout
#include <limits> // -inf
#include <iterator> // std::distance

// for debugging only
#include <time.h> // for srand

#include "actionSelection.h"
#include "logUtils.h"
#include "translator.h"
#include "modelUtils.h"
#include "setupUtils.h"

#define _USE_MATH_DEFINES
#include <math.h> // cos, sin

// for debugging only
double timeDiffa(timespec& ts1, timespec& ts2){
  return (double) ts2.tv_sec - (double) ts1.tv_sec + ((double) ts2.tv_nsec - (double) ts1.tv_nsec)/1000000000; 
}

////////////////////////////////////////////////////////////////////////////////
//                                New Section                                 //
////////////////////////////////////////////////////////////////////////////////

// !!!!!!!!!!!!!!!!!!!!!!!! ALL ACTIONS ARE RELATIVE !!!!!!!!!!!!!!!!!!!!!!!! //

// Try to implement the OG version for particle filters
void actionSelection::chooseActionOGPart(std::vector<BayesFilter>& filterBank,
					 std::vector<double>& fbProbs,
					 std::vector<stateStruct>& bestStates,
					 std::vector< std::vector<double> >& 
					 actionList,
					 std::vector<double>& action,
					 std::vector<double>& poseInRbt,
					 std::vector< std::vector<double> >& 
					 workspace){
  //OG == Original Gangster
  //The original method was simply to run a bunch of actions on the top two models in terms of probability and try to determine which action produced the most different outcome and then choose that one.
  //this will only work if there is at least two models
  //and the probabilities better be positive


  // Step -1: check if you're uniform first because this doesn't make sense 
  // if you are

  double tol = 0.001;
  double uniformFilterProb = 1.0/fbProbs.size();
  bool uniformPrior = true;
  for(size_t i=0;i<fbProbs.size();i++){
    if(fbProbs[i] < (uniformFilterProb-tol) || 
       fbProbs[i] > (uniformFilterProb+tol)){
      uniformPrior = false;
      break;
    }
  }
  
  if(uniformPrior){
    std::cout << "Choose randomly because distribution is uniform" << std::endl;
    chooseActionRandomRel(actionList,action,poseInRbt,workspace);
  }
  else{
    //Step 0: Validate relative action list
    std::vector< std::vector<double> > validRelActionList;
    validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

    //Step 1: Calculate all the filter probabilities so you know which models to compare (the highest two)

    // you already have this from the previous step calculation

    //std::vector<double> modelProbs = calcModelProb();
    //std::vector<double> fbProbs = modelUtils::calcFilterBankProbs(filterBank);

    if (fbProbs.size()<2){
      std::cout << "At least 2 models needed" << std::endl;
    }
    else {

      //std::cout << "Let's choose a model" << std::endl;
      double first=-1.0;
      double second=-1.0;
      int firstIndex=-1;
      int secondIndex=-1;
    
      for (size_t i = 0; i<fbProbs.size(); i++){
	if (fbProbs[i]>first){
	  second = first;
	  secondIndex = firstIndex;
	  first = fbProbs[i];
	  firstIndex = i;
	}
	else if (fbProbs[i]>second){
	  second = fbProbs[i];
	  secondIndex = i;
	}
      }

      //int firstModel = modelParamPairs[firstIndex].model;
      //int secondModel = modelParamPairs[secondIndex].model;

      //std::vector<double> firstParams = modelParamPairs[firstIndex].params;
      //std::vector<double> secondParams = modelParamPairs[secondIndex].params;
    
      // Step 2: Figure out the maximum probability state for those models to simulate from 
      // (THIS IS IN LOG SPACE)
      stateStruct firstState = bestStates[firstIndex];
      //double firstStateProb;
      stateStruct secondState = bestStates[secondIndex];
      //double secondStateProb;
      //bool foundFirst = false;
      //bool foundSecond = false;

      /*
	for (size_t i = 0; i<filter.stateList_.size(); i++){
	if (filter.stateList_[i].model == firstModel && filter.stateList_[i].params == firstParams){
	if (foundFirst == false){
	firstState = filter.stateList_[i];
	firstStateProb = filter.logProbList_[i];
	foundFirst = true;
	}
	else if (filter.logProbList_[i] > firstStateProb){
	firstState = filter.stateList_[i];
	firstStateProb = filter.logProbList_[i];
	}
	}
	else if (filter.stateList_[i].model == secondModel && filter.stateList_[i].params == secondParams){
	if (foundSecond == false){
	secondState = filter.stateList_[i];
	secondStateProb = filter.logProbList_[i];
	foundSecond = true;
	}
	else if (filter.logProbList_[i] > secondStateProb){
	secondState = filter.stateList_[i];
	secondStateProb = filter.logProbList_[i];
	}
	}
	}
      */

      //Step 3: Simulate all the actions from the states found in Step 2 + 
      //Step 4: Calculate distances between results from Step 3 and determine which is greatest
      stateStruct tempFirstNextState;
      stateStruct tempSecondNextState;
      double furthestDist = -1.0;
      double currentDist = -2.0;
      std::vector<double> bestAction;
    
      //std::cout << "error is righhhhhhht here:" << std::endl;


      for (size_t i=0; i<validRelActionList.size(); i++){
	tempFirstNextState = 
	  translator::stateTransition(firstState,validRelActionList[i]);
	tempSecondNextState = 
	  translator::stateTransition(secondState,validRelActionList[i]);

	currentDist = 
	  distPointsSquared(translator::translateStToObs(tempFirstNextState),
			    translator::translateStToObs(tempSecondNextState));

	if (currentDist > furthestDist){
	  furthestDist = currentDist;
	  bestAction = validRelActionList[i];
	}
      }

      //std::cout << "def not here" << std::endl;

      //Step 5: Set the next action to do to the best action found
      action = bestAction;
    }
  }
}


// Use a distance metric to choose the next action
void actionSelection::chooseActionPartDist(std::vector<BayesFilter>& filterBank,std::vector< std::vector<double> >& actionList,std::vector<double>& action,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace){

  // This method finds the MAP estimate and sees which action will cause it to 
  // move the most.

  // Step -2: Validate relative action list
  std::vector< std::vector<double> > validRelActionList;
  validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

  // Step -1: Find the MAP estimate
  size_t maxInd;
  size_t filterInd;
  double maxProb = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i<filterBank.size(); i++){
    std::vector<double>::iterator maxIt = 
      std::max_element(filterBank[i].logProbList_.begin(),
		       filterBank[i].logProbList_.end());
    double maxProbNew = *maxIt;
    if (maxProb < maxProbNew){
      maxProb = maxProbNew;
      maxInd = std::distance(filterBank[i].logProbList_.begin(),maxIt);
      filterInd = i;
    }
  }

  stateStruct maxState = filterBank[filterInd].stateList_[maxInd];
  std::cout << "max state model:" << maxState.model << std::endl;
  std::cout << "max state prob:" << maxProb << std::endl;
  // Step 2: For each action, calculate the distance the MAP estimate will move
  std::vector<double> distsSquaredList; 

  for (size_t i = 0; i<validRelActionList.size(); i++){

    stateStruct nextState = 
      translator::stateTransition(maxState, validRelActionList[i]);

    std::vector<double> newPoseInRbt = translator::translateStToObs(nextState);

    distsSquaredList.push_back(distPointsSquared(poseInRbt,newPoseInRbt));

  }

  //Step 3: Choose the action which results in the lowest entropy updated belief
  std::vector<double>::iterator maxDistSquaredIt = 
    std::max_element(distsSquaredList.begin(),distsSquaredList.end());
  action = 
    validRelActionList[std::distance(distsSquaredList.begin(),
				     maxDistSquaredIt)];

}

// Use a distance metric to choose the next action
void actionSelection::chooseActionPartDist2(std::vector<BayesFilter>& filterBank,std::vector< std::vector<double> >& actionList,std::vector<double>& action,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace){

  // The new method samples from the belief state according to the probability 
  // distribution and simulates from those states for each action. An 
  // observation is taken for the output state after the simulation. Given the 
  // observation, the belief state is updated. The probability distribution over
  // models is calculated. The entropy of this distribution is calculated. The 
  // action which produces the lowest entropy is chosen.


  // Step -2: Validate relative action list
  std::vector< std::vector<double> > validRelActionList;
  validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

  // Step 1: Accumulate states and probs from bank into single vectors
  // Calculate total number of states
  size_t totalNumStates = 0;
  for (size_t i = 0; i<filterBank.size(); i++){
    totalNumStates += filterBank[i].stateList_.size();
  }
  // Initialize total vectors
  std::vector<stateStruct> totalStateList;
  std::vector<double> totalLogProbList;
  // Reserve the right amount of space
  totalStateList.reserve(totalNumStates);
  totalLogProbList.reserve(totalNumStates);
  // Insert the vectors
  for (size_t i = 0; i<filterBank.size(); i++){
    totalStateList.insert(totalStateList.end(),
			  filterBank[i].stateList_.begin(),
			  filterBank[i].stateList_.end());

    totalLogProbList.insert(totalLogProbList.end(),
			  filterBank[i].logProbList_.begin(),
			  filterBank[i].logProbList_.end());
  }
  


  // Step 0: Assume only log probs exist. Exponentiate to get probs.
  std::vector<double> probList = logUtils::expLogProbs(totalLogProbList);

  // Step 1: Create the CDF of the current belief from the PDF probList_.
  std::vector<double> probCDF = createCDF(probList);
  

  // Step 2: For each action, sample a state from the belief n times. Simulate 
  // this state with the action and get an observation. Update the belief with 
  // the action-observation pair. Calculate the entropy of the new belief. 
  // Average the entropies over the n samples.


  // this is a list of average entropies, one for each action
  std::vector<double> avgDistList; 
  int nSamples = 4000; //number of samples of the belief state per action
  //std::cout << "samples" << nSamples << std::endl;
  for (size_t i = 0; i<validRelActionList.size(); i++){

    std::vector<double> distList; //this is per action

    for (size_t j = 0; j<nSamples; j++){

      // Step 2.1: Sample a nextState from the belief over transitioned states
      stateStruct sample = getSampleState(probCDF,totalStateList);
      // Step 2.2: Simulate the state with the action
      // Step 2.3: Get an observation
      // Step 2.4: Update the belief state in log space

      stateStruct nextState = 
	translator::stateTransition(sample, validRelActionList[i]);
      
      std::vector<double> newPoseInRbt = 
	translator::translateStToObs(nextState);

      distList.push_back(distPointsSquared(poseInRbt,newPoseInRbt));

    }
    // Step 2.7: Average the entropies over the n samples
    double dSum = 
      std::accumulate(distList.begin(),distList.end(),(double) 0.0);
    avgDistList.push_back(dSum/distList.size());
  }

  //Step 3: Choose the action which results in the lowest entropy updated belief
  std::vector<double>::iterator maxAvgDistIt = 
    std::max_element(avgDistList.begin(),avgDistList.end());
  action = 
    validRelActionList[std::distance(avgDistList.begin(),maxAvgDistIt)];

}

// Use an entropy metric to choose the next action
void actionSelection::chooseActionPartEntropy(std::vector<BayesFilter>& filterBank,std::vector< std::vector<double> >& actionList,std::vector<double>& action,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace){

  timespec ts9;
  timespec ts10;
  timespec ts11;
  timespec ts12;
  timespec ts13;
  timespec ts14;
  timespec ts15;
  timespec ts16;
  timespec ts17;
  timespec ts18;
  timespec ts19;
  timespec ts20;
  timespec ts21;
  timespec ts22;
  timespec ts23;
  timespec ts24;

  // The new method samples from the belief state according to the probability 
  // distribution and simulates from those states for each action. An 
  // observation is taken for the output state after the simulation. Given the 
  // observation, the belief state is updated. The probability distribution over
  // models is calculated. The entropy of this distribution is calculated. The 
  // action which produces the lowest entropy is chosen.

  timing::get_time(ts9);

  // Step -2: Validate relative action list
  std::vector< std::vector<double> > validRelActionList;
  validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

  timing::get_time(ts10);

  // Step 1: Accumulate states and probs from bank into single vectors
  // Calculate total number of states
  size_t totalNumStates = 0;
  for (size_t i = 0; i<filterBank.size(); i++){
    totalNumStates += filterBank[i].stateList_.size();
  }
  // Initialize total vectors
  std::vector<stateStruct> totalStateList;
  std::vector<double> totalLogProbList;
  // Reserve the right amount of space
  totalStateList.reserve(totalNumStates);
  totalLogProbList.reserve(totalNumStates);
  // Insert the vectors
  for (size_t i = 0; i<filterBank.size(); i++){
    totalStateList.insert(totalStateList.end(),
			  filterBank[i].stateList_.begin(),
			  filterBank[i].stateList_.end());

    totalLogProbList.insert(totalLogProbList.end(),
			  filterBank[i].logProbList_.begin(),
			  filterBank[i].logProbList_.end());
  }

  timing::get_time(ts11);


  // Step 0: Assume only log probs exist. Exponentiate to get probs.
  std::vector<double> probList = logUtils::expLogProbs(totalLogProbList);

  // Step 1: Create the CDF of the current belief from the PDF probList_.
  std::vector<double> probCDF = createCDF(probList);

  // Step 2: For each action, sample a state from the belief n times. Simulate 
  // this state with the action and get an observation. Update the belief with 
  // the action-observation pair. Calculate the entropy of the new belief. 
  // Average the entropies over the n samples.

  timing::get_time(ts12);

  // this is a list of average entropies, one for each action
  std::vector<double> avgEntropyList; 
  int nSamples = 12; //number of samples of the belief state per action
  //std::cout << "samples" << nSamples << std::endl;
  for (size_t i = 0; i<validRelActionList.size(); i++){

    timing::get_time(ts13);

    std::vector<double> entropyList; //this is per action

    // We only need to do the transition update once per action and do the
    // observation update for every sample taken under that action.

    // Create a copy of the totalStateList to use for each action
    std::vector<stateStruct> localStateList = totalStateList;

    // For now, change answers to closed form inside stateTransition
    // Transition the states
    // Hijack the first filter to use its class functions
    filterBank[0].transitionUpdateLog(localStateList,validRelActionList[i]);


    for (size_t j = 0; j<nSamples; j++){

      timing::get_time(ts14);

      // Step 2.0: Create a copy of the real probability list
      // only for this action and sample
      std::vector<double> localLogProbList = totalLogProbList;

      timing::get_time(ts19);

      // Step 2.1: Sample a nextState from the belief over transitioned states
      stateStruct nextState = getSampleState(probCDF,localStateList);

      timing::get_time(ts24);

      // Step 2.2: Simulate the state with the action
      
      timing::get_time(ts20);

      // Step 2.3: Get an observation
      // Step 2.4: Update the belief state in log space

      timing::get_time(ts21);

      filterBank[0].observationUpdateLog(localLogProbList,localStateList,
				  getNoisyObs(nextState));

      timing::get_time(ts22);
      
      // Step 2.5: Normalize probability over full prob list (across the bank)
      // which is no longer done in the observation update
      localLogProbList = logUtils::normalizeVectorInLogSpace(localLogProbList);

      timing::get_time(ts23);

      // Step 2.6: Calculate the entropy over models of the new belief state
      entropyList.push_back(calcEntropy(logUtils::expLogProbs(localLogProbList)));

      timing::get_time(ts15);      

    }
    // Step 2.7: Average the entropies over the n samples
    double eSum = 
      std::accumulate(entropyList.begin(),entropyList.end(),(double) 0.0);
    avgEntropyList.push_back(eSum/entropyList.size());

    // DELETE
    /*
    std::cout << "entropyList: ";
    for (size_t jj=0;jj<entropyList.size();jj++){
      std::cout << entropyList[jj] << ",";
    }
    std::cout << std::endl;
    */

    timing::get_time(ts16);

  }
  
  // DELETE
  /*
  std::cout << "avgEntropyList: ";
  for (size_t jj=0;jj<avgEntropyList.size();jj++){
    std::cout << avgEntropyList[jj] << ",";
  }
  std::cout << std::endl;
  */


  timing::get_time(ts17);


  //Step 3: Choose the action which results in the lowest entropy updated belief
  std::vector<double>::iterator minAvgEntIt = 
    std::min_element(avgEntropyList.begin(),avgEntropyList.end());
  action = 
    validRelActionList[std::distance(avgEntropyList.begin(),minAvgEntIt)];

  timing::get_time(ts18);
  
  std::cout << "Time to validate:\n" << timeDiffa(ts9,ts10) << std::endl;
  std::cout << "Time to accumulate:\n" << timeDiffa(ts10,ts11) << std::endl;
  std::cout << "Time to create CDF:\n" << timeDiffa(ts11,ts12) << std::endl;
  std::cout << "Time for one action:\n" << timeDiffa(ts13,ts16) << std::endl;
  std::cout << "Time for one sample:\n" << timeDiffa(ts14,ts15) << std::endl;
  std::cout << "Time to find best:\n" << timeDiffa(ts17,ts18) << std::endl;

  std::cout << "Time to copy:\n" << timeDiffa(ts14,ts19) << std::endl;
  std::cout << "Time to sample and sim:\n" << timeDiffa(ts19,ts20) << std::endl;
  std::cout << "Time to sample:\n" << timeDiffa(ts19,ts24) << std::endl;
  std::cout << "Time to sim:\n" << timeDiffa(ts24,ts20) << std::endl;
  std::cout << "Time to transition:\n" << timeDiffa(ts20,ts21) << std::endl;
  std::cout << "Time to observe:\n" << timeDiffa(ts21,ts22) << std::endl;
  std::cout << "Time to normalize:\n" << timeDiffa(ts22,ts23) << std::endl;
  std::cout << "Time to calc entropy:\n" << timeDiffa(ts23,ts15) << std::endl;


}


////////////////////////////////////////////////////////////////////////////////
//                              End New Section                               //
////////////////////////////////////////////////////////////////////////////////


// Simplest action selection. Just go through the list.
void actionSelection::chooseActionSimple(std::vector< std::vector<double> >& actionList,int step,std::vector<double>& action){
  action = actionList[step%actionList.size()];
}

// Random action selection.
void actionSelection::chooseActionRandom(std::vector< std::vector<double> >& actionList,std::vector<double>& action){
  size_t ind = rand() % actionList.size();
  action = actionList[ind];
}



////////////////////////////////////////////////////////////////////////////////
//                             Relative Section                               //
////////////////////////////////////////////////////////////////////////////////

//std::vector<double> poseInRbt should be what is passed

// Simplest action selection. Just go through the list. Doesn't really make sense
void actionSelection::chooseActionSimpleRel(std::vector< std::vector<double> >& actionList,int step,std::vector<double>& action,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace){

  std::vector< std::vector<double> > validRelActionList;
  validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

  action = validRelActionList[step%validRelActionList.size()]; // select action
  
  // DELETE THIS
  //action = actionList[7];
  //if (step > 2) action = actionList[3];

  /*
  if (step == 0) action = actionList[0];
  else if (step == 1) action = actionList[0];
  else if (step == 2) action = actionList[2];
  else if (step == 3) action = actionList[2];
  else if (step == 4) action = actionList[4];
  else if (step == 5) action = actionList[4];
  else if (step == 6) action = actionList[6];
  else if (step == 7) action = actionList[6];
  else if (step == 8) action = actionList[0];
  else if (step == 9) action = actionList[0];
  else action = actionList[0];
  */

  /*
  if (step == 0) action = actionList[0];
  else if (step == 1) action = actionList[0];
  else if (step == 2) action = actionList[2];
  else if (step == 3) action = actionList[4];
  else if (step == 4) action = actionList[4];
  else if (step == 5) action = actionList[6];
  else if (step == 6) action = actionList[4];
  else if (step == 7) action = actionList[4];
  else if (step == 8) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // revolute joint perfect
  if (step == 0) action = actionList[7];
  else if (step == 1) action = actionList[7];
  else if (step == 2) action = actionList[0];
  else if (step == 3) action = actionList[4];
  else if (step == 4) action = actionList[3];
  else if (step == 5) action = actionList[3];
  else if (step == 6) action = actionList[3];
  else if (step == 7) action = actionList[2];
  else if (step == 8) action = actionList[2];
  else if (step == 9) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // revolute joint not perfect
  if (step == 0) action = actionList[7];
  else if (step == 1) action = actionList[1];
  else if (step == 2) action = actionList[7];
  else if (step == 3) action = actionList[0];
  else if (step == 4) action = actionList[6];
  else if (step == 5) action = actionList[4];
  else if (step == 6) action = actionList[3];
  else if (step == 7) action = actionList[3];
  else if (step == 8) action = actionList[5];
  else if (step == 9) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // free vs latch - to cover the space
  if (step == 0) action = actionList[7];
  else if (step == 1) action = actionList[2];
  else if (step == 2) action = actionList[3];
  else if (step == 3) action = actionList[4];
  else if (step == 4) action = actionList[6];
  else if (step == 5) action = actionList[6];
  else if (step == 6) action = actionList[0];
  else if (step == 7) action = actionList[0];
  else if (step == 8) action = actionList[2];
  else if (step == 9) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // optimal free?
  if (step == 0) action = actionList[7];
  else if (step == 1) action = actionList[2];
  else if (step == 2) action = actionList[3];
  else if (step == 3) action = actionList[4];
  else if (step == 4) action = actionList[6];
  else if (step == 5) action = actionList[6];
  else if (step == 6) action = actionList[0];
  else if (step == 7) action = actionList[0];
  else if (step == 8) action = actionList[2];
  else if (step == 9) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // optimal fixed?
  if (step == 0) action = actionList[0];
  else if (step == 1) action = actionList[4];
  else if (step == 2) action = actionList[2];
  else if (step == 3) action = actionList[6];
  else if (step == 4) action = actionList[1];
  else if (step == 5) action = actionList[5];
  else if (step == 6) action = actionList[3];
  else if (step == 7) action = actionList[7];
  else if (step == 8) action = actionList[0];
  else if (step == 9) action = actionList[4];
  else action = actionList[0];
  */

  /*
  // optimal rev?
  if (step == 0) action = actionList[7];
  else if (step == 1) action = actionList[3];
  else if (step == 2) action = actionList[3];
  else if (step == 3) action = actionList[7];
  else if (step == 4) action = actionList[7];
  else if (step == 5) action = actionList[6];
  else if (step == 6) action = actionList[2];
  else if (step == 7) action = actionList[3];
  else if (step == 8) action = actionList[3];
  else if (step == 9) action = actionList[7];
  else action = actionList[0];
  */

  /*
  // optimal pris?
  if (step == 0) action = actionList[6];
  else if (step == 1) action = actionList[0];
  else if (step == 2) action = actionList[4];
  else if (step == 3) action = actionList[2];
  else if (step == 4) action = actionList[2];
  else if (step == 5) action = actionList[0];
  else if (step == 6) action = actionList[4];
  else if (step == 7) action = actionList[6];
  else if (step == 8) action = actionList[6];
  else if (step == 9) action = actionList[2];
  else action = actionList[0];
  */

  /*
  // optimal latch?
  if (step == 0) action = actionList[4];
  else if (step == 1) action = actionList[0];
  else if (step == 2) action = actionList[2];
  else if (step == 3) action = actionList[4];
  else if (step == 4) action = actionList[4];
  else if (step == 5) action = actionList[6];
  else if (step == 6) action = actionList[6];
  else if (step == 7) action = actionList[0];
  else if (step == 8) action = actionList[6];
  else if (step == 9) action = actionList[4];
  else action = actionList[0];
  */


}

// Random action selection.
void actionSelection::chooseActionRandomRel(std::vector< std::vector<double> >& actionList,std::vector<double>& action,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace){

  std::vector< std::vector<double> > validRelActionList;
  validateRelActionList(actionList,poseInRbt,workspace,validRelActionList);

  if (validRelActionList.size()>0){
    size_t ind = rand() % validRelActionList.size();
    action = validRelActionList[ind];
  }
  else{
    double x;
    double y;
    std::vector<double> dists;
    for(size_t i=0;i<actionList.size();i++){
      x = poseInRbt[0]+actionList[i][0];
      y = poseInRbt[1]+actionList[i][1];
      dists.push_back(x*x+y*y);
    }
    action = actionList[std::distance(dists.begin(),
				      std::min_element(dists.begin(),
						       dists.end()))];
  }
}

////////////////////////////////////////////////////////////////////////////////
//                           End Relative Section                             //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                               Aux Section                                  //
////////////////////////////////////////////////////////////////////////////////

std::vector<double> actionSelection::getNoisyObs(stateStruct& state){
  // maybe do something about srand later. not calling now on purpose.
  std::vector<double> obs = translator::translateStToObs(state);
  // add some noise
  for (size_t i=0; i<obs.size(); i++){
    //double X = randomDouble();
    //obs[i]+=0.001*X; // add noise to each element
    obs[i]+=gaussianNoise(); // add noise to each element
  }
  return obs;
}

std::vector<double> actionSelection::createCDF(std::vector<double>& probList){
  // this takes probabilities in regular space (not log space)
  std::vector<double> probCDF;
  probCDF.push_back(probList[0]); //initialize the first entry for the list
  for (size_t i=1; i<probList.size(); i++){
    probCDF.push_back(probCDF[i-1]+probList[i]);
  }
  return probCDF;
}

stateStruct actionSelection::getSampleState(std::vector<double>& CDF, std::vector<stateStruct>& states){
  // uniformly sample a state
  double X = randomDouble();
  std::vector<double>::iterator low = std::lower_bound(CDF.begin(),CDF.end(),X);
  return states[std::distance(CDF.begin(),low)];
}

double actionSelection::calcEntropy(std::vector<double> probs){
  // this takes probabilities in regular space (not log space)
  double sum=0;
  for (size_t i=0; i<probs.size(); i++){
    if (probs[i]!=0.0){
      sum += probs[i]*log(probs[i]);
    }
  }
  return -sum;
}

double actionSelection::randomDouble(){
  double X = ((double)rand()/(double)RAND_MAX);
  return X;
}

double actionSelection::gaussianNoise(){
  double x1 = ((double)rand()/(double)RAND_MAX);
  double x2 = ((double)rand()/(double)RAND_MAX);
  double sig = 0.01; // standard deviation of noise - it worked when it was 0.00001 - still worked with 0.01
  double mu = 0.0; // mean of noise
  return sqrt(-2*logUtils::safe_log(x1))*cos(2*M_PI*x2)*sig+mu;
}

double actionSelection::distPointsSquared(std::vector<double> a, std::vector<double> b){
  //assumes 2d distances
  return (a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]);
}

void actionSelection::relToAbsActionList(std::vector< std::vector<double> >& relActionList,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& absActionList){
  absActionList.clear(); // Clear anything in here
  std::vector<double> tempAction;
  for (size_t i=0;i<relActionList.size();i++){
    for (size_t j=0;j<poseInRbt.size();j++){
      tempAction.push_back(poseInRbt[j]+relActionList[i][j]);
    }
    absActionList.push_back(tempAction);
  }
}

void actionSelection::absToRelActionList(std::vector< std::vector<double> >& absActionList,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& relActionList){
  relActionList.clear(); // Clear anything in here
  std::vector<double> tempAction;
  for (size_t i=0;i<absActionList.size();i++){
    for (size_t j=0;j<poseInRbt.size();j++){
      tempAction.push_back(absActionList[i][j]-poseInRbt[j]);
    }
    relActionList.push_back(tempAction);
  }
}

void actionSelection::absToRelAction(std::vector<double>& tempAbsAction,std::vector<double>& poseInRbt,std::vector<double>& tempRelAction){
  tempRelAction.clear();
  for (size_t i=0;i<tempAbsAction.size();i++){
    tempRelAction.push_back(tempAbsAction[i]-poseInRbt[i]);
  }
}


void actionSelection::validateRelActionList(std::vector< std::vector<double> >& relActionList,std::vector<double>& poseInRbt,std::vector< std::vector<double> >& workspace,std::vector< std::vector<double> >& validRelActionList){

  /*
  std::vector< std::vector<double> > validAbsActionList;
  //std::vector<double> stateInRbt = translator::translateStToRbt(state); // Convert state to rbt coordinates

  relToAbsActionList(relActionList,poseInRbt,validAbsActionList); // convert rel to abs actions
  setupUtils::validateActions(validAbsActionList,workspace); // validate actions: action list is mutated to only be valid actions
  absToRelActionList(validAbsActionList,poseInRbt,validRelActionList); // convert abs to rel actions

  // All workspace stuff right now assumes a 2D workspace - FIX
  // Actions must be within the workspace
  */

  validRelActionList.clear();
  for (size_t i=0;i<relActionList.size();i++){
    if (!((poseInRbt[0]+relActionList[i][0])<workspace[0][0] || 
	  (poseInRbt[0]+relActionList[i][0])>workspace[0][1] || 
	  (poseInRbt[1]+relActionList[i][1])<workspace[1][0] || 
	  (poseInRbt[1]+relActionList[i][1])>workspace[1][1])){
      validRelActionList.push_back(relActionList[i]);
    }
  }

}


////////////////////////////////////////////////////////////////////////////////
//                             End Aux Section                                //
////////////////////////////////////////////////////////////////////////////////

