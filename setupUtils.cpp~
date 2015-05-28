//Setup Utilities
#include "setupUtils.h"
#include "logUtils.h"
#include "translator.h"
#include "actionSelection.h"
#include "fileUtils.h"

#include "globalVars.h" // Global variables

#include <iostream> // DELETE
#define _USE_MATH_DEFINES
#include <math.h> // cos, sin
#include <algorithm> // std::find_if

#include <stdexcept> // throw exception

bool closeEnough(std::vector<double> &a1,std::vector<double> &a2,double thresh){
  return pow(a1[0]-a2[0],2)<pow(thresh,2) && pow(a1[1]-a2[1],2)<pow(thresh,2);
}

// fake actions and fake observations

void setupUtils::fakeAOfromFile(std::vector<std::vector<double> > &actionList,
				std::vector<int> &FAinds,
				std::vector<std::vector<double> > &fakeObs,
				std::string fileName,
				int numSteps){
  std::vector<std::vector<double> > fakeActions;
  fileUtils::txtFileToActionsObs(fileName,fakeActions,fakeObs,numSteps);

  std::cout << "action size: " << fakeActions.size() << std::endl;
  std::cout << "obs size:" << fakeObs.size() << std::endl;

  std::vector<int> FAIs;
  double thresh = 0.00001;
  for(size_t i=0;i<fakeActions.size();i++){
    std::cout << "step: " << i << std::endl;
    std::cout << fakeActions[i][0] << "," << fakeActions[i][1] << std::endl;
    for(size_t j=0;j<actionList.size();j++){
      std::cout << actionList[j][0] << "," << actionList[j][1] << std::endl;
      if(closeEnough(fakeActions[i],actionList[j],thresh)){
	std::cout << "found an action" << std::endl;
	FAIs.push_back(j);
	break;
      }
      else std::cout << "didn't match that action" << std::endl;
    }
  }
  FAinds = FAIs;
}

// fake actions

std::vector<int> setupUtils::fakeActions(std::vector< 
					   std::vector<double> >& actionList){
  std::vector<std::vector<double> > fakeActions;

  std::vector<double> actions0;
  actions0.push_back(-2.20436e-17);
  actions0.push_back(-0.12);
  fakeActions.push_back(actions0);
  std::vector<double> actions1;
  actions1.push_back(0.12);
  actions1.push_back(0.0);
  fakeActions.push_back(actions1);
  std::vector<double> actions2;
  actions2.push_back(0.0848528);
  actions2.push_back(0.0848528);
  fakeActions.push_back(actions2);
  std::vector<double> actions3;
  actions3.push_back(-0.12);
  actions3.push_back(1.46958e-17);
  fakeActions.push_back(actions3);
  std::vector<double> actions4;
  actions4.push_back(-2.20436e-17);
  actions4.push_back(-0.12);
  fakeActions.push_back(actions4);
  std::vector<double> actions5;
  actions5.push_back(0.0848528);
  actions5.push_back(0.0848528);
  fakeActions.push_back(actions5);
  std::vector<double> actions6;
  actions6.push_back(0.0848528);
  actions6.push_back(-0.0848528);
  fakeActions.push_back(actions6);
  std::vector<double> actions7;
  actions7.push_back(0.12);
  actions7.push_back(0.0);
  fakeActions.push_back(actions7);
  std::vector<double> actions8;
  actions8.push_back(-0.0848528);
  actions8.push_back(-0.0848528);
  fakeActions.push_back(actions8);
  std::vector<double> actions9;
  actions9.push_back(7.34788e-18);
  actions9.push_back(0.12);
  fakeActions.push_back(actions9);

  std::vector<int> FAinds;
  double thresh = 0.00001;
  for(size_t i=0;i<fakeActions.size();i++){
    for(size_t j=0;j<actionList.size();j++){
      if(closeEnough(fakeActions[i],actionList[j],thresh)){
	FAinds.push_back(j);
	break;
      }
    }
  }
  return FAinds;
}

// fake observations

std::vector<std::vector<double> > setupUtils::fakeObs(){
  std::vector<std::vector<double> > fakeObs;

  std::vector<double> obs0;
  obs0.push_back(0.00061114);
  obs0.push_back(0.002612);
  fakeObs.push_back(obs0);
  std::vector<double> obs1;
  obs1.push_back(0.00133904);
  obs1.push_back(0.00395064);
  fakeObs.push_back(obs1);
  std::vector<double> obs2;
  obs2.push_back(0.00185868);
  obs2.push_back(0.00609914);
  fakeObs.push_back(obs2);
  std::vector<double> obs3;
  obs3.push_back(0.000326171);
  obs3.push_back(0.00410389);
  fakeObs.push_back(obs3);
  std::vector<double> obs4;
  obs4.push_back(0.000480639);
  obs4.push_back(0.00303153);
  fakeObs.push_back(obs4);
  std::vector<double> obs5;
  obs5.push_back(0.00150034);
  obs5.push_back(0.00635989);
  fakeObs.push_back(obs5);
  std::vector<double> obs6;
  obs6.push_back(0.0010297);
  obs6.push_back(0.00337847);
  fakeObs.push_back(obs6);
  std::vector<double> obs7;
  obs7.push_back(0.00143139);
  obs7.push_back(0.00413988);
  fakeObs.push_back(obs7);
  std::vector<double> obs8;
  obs8.push_back(9.53211e-05);
  obs8.push_back(0.00229638);
  fakeObs.push_back(obs8);
  std::vector<double> obs9;
  obs9.push_back(0.00151811);
  obs9.push_back(0.0034373);
  fakeObs.push_back(obs9);

  return fakeObs;
}


////////////////////////////////////////////////////////////////////////////////
//                            Particle Section                                //
////////////////////////////////////////////////////////////////////////////////

// SAMPLING SECTION

using Eigen::MatrixXd;
using Eigen::VectorXd;

void setupUtils::resampleParticles(std::vector<stateStruct>& stateList,std::vector<double>& logProbList){
  double logProbPerPart = logUtils::logSumExp(logProbList)
    -logUtils::safe_log(logProbList.size());

  std::cout << "Number of particles: " << logProbList.size() << std::endl;
  std::cout << "Log Prob Per Particle: " << logProbPerPart << std::endl;

  // Samples from the particles state according to the probability distribution 
  // Step 0: Normalize the log probability list
  std::vector<double> normLogProbList = 
    logUtils::normalizeVectorInLogSpace(logProbList);

  //Step 1: Assume only log probs exist. Exponentiate to get probs.
  std::vector<double> probList = logUtils::expLogProbs(normLogProbList);

  //Step 1: Create the CDF of the current belief from the PDF probList_.
  std::vector<double> probCDF = actionSelection::createCDF(probList);
  std::cout << probCDF[0] << std::endl;
  std::cout << probCDF[probCDF.size()-1] << std::endl;

  //Step 2: Sample states from the belief
  std::vector<stateStruct> tempStateList; // empty state list
  for (size_t i=0;i<stateList.size();i++){
    tempStateList.push_back(actionSelection::getSampleState(probCDF,stateList));
  }
  stateList = tempStateList;
  std::vector<double> tempLogProbList (logProbList.size(),logProbPerPart);
  logProbList = tempLogProbList;
}

std::vector<double> setupUtils::standardGaussianVariates(){
  // Box-Muller Transform
  // Generate random numbers between 0 and 1
  double x1 = ((double)rand()/(double)RAND_MAX);
  double x2 = ((double)rand()/(double)RAND_MAX);
  // Calculate standard normal variates (mean 0, var 1)
  std::vector<double> variates;
  variates.push_back(sqrt(-2*logUtils::safe_log(x1))*cos(2*M_PI*x2));
  variates.push_back(sqrt(-2*logUtils::safe_log(x1))*sin(2*M_PI*x2));
  return variates;
}

double setupUtils::randomDouble(){
  double X = ((double)rand()/(double)RAND_MAX);
  return X;
}

Eigen::VectorXd setupUtils::sampleParticle(unsigned int size,Eigen::VectorXd& mu,Eigen::MatrixXd& A){
  // Sample size standard normal variates
  Eigen::VectorXd z = VectorXd::Zero(size);
  for (size_t i=0; i<size; i+=2){
    std::vector<double> variates = standardGaussianVariates();
    z[i] = variates[0];
    if (i != size-1) z[i+1] = variates[1];
  }
  // Return sample from multivariate guassian (type Eigen::VectorXd x)
  return  mu+A*z;
}

void setupUtils::setupParticles(std::vector<stateStruct>& stateList,std::vector<double>& logProbList,int modelNum,double initParamVar,double initVarVar,int numParticles,int numMechTypes,std::vector< std::vector<double> >& workspace){
  stateList.clear(); // Make sure the stateList is empty
  if (modelNum == 1){
    // For model 1 (the fixed model), only ever sample the single valid state
    for (size_t i=0;i<numParticles;i++){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(0.0);
      x_state.params.push_back(0.0);
      stateList.push_back(x_state);
    }
  }
  else{
    // For any other model:
    // Determine shape of state space for this modelNum
    int numParams = MODEL_DESCRIPTIONS[modelNum][0]; // in globalVars.h
    int numVars = MODEL_DESCRIPTIONS[modelNum][1]; // in globalVars.h
    unsigned int size = numParams+numVars; // dimension of space
    Eigen::MatrixXd Cov = MatrixXd::Zero(size,size);
    Eigen::VectorXd mu = VectorXd::Zero(size);
    // Setup initial diagonal covariance matrix with
    // a) initial parameter variance for each parameter dimension
    // b) initial variable variance for each variable dimension
    for (size_t i=0; i<numParams; i++){
      Cov(i,i) = initParamVar;
    }
    for (size_t i=numParams; i<size; i++){
      Cov(i,i) = initVarVar;
    }
    // Generate matrix A from Choleksy decomposition of Cov Matrix
    // Cast back to dense matrix for use in sampling function
    Eigen::MatrixXd A( Cov.llt().matrixL() );
    // Sample numParticles particles, 
    // shape them into states, and place in vector
    size_t count = 0; // How many valid particles we have sampled
    while (count<numParticles){
      Eigen::VectorXd x = sampleParticle(size,mu,A);
      stateStruct x_state;
      x_state.model = modelNum;
      for (size_t i=0; i<numParams; i++){
	x_state.params.push_back(x(i));
      }
      for (size_t i=numParams; i<size; i++){
	x_state.vars.push_back(x(i));
      }
      if (translator::isStateValid(x_state,workspace)){
	stateList.push_back(x_state);
	count++;
      }
    }
  }
  // Set the probability of the samples equal across all models
  // prob per particle
  double probPerParticle = logUtils::safe_log(1.0/(numParticles*numMechTypes));
  logProbList.clear(); // Make sure the logProbList is empty
  std::vector<double> probs (numParticles,probPerParticle);
  logProbList = probs;
}

void setupUtils::setupParticlesIndependent(std::vector<stateStruct>& stateList,std::vector<double>& logProbList,int modelNum,double initParamVar,double initVarVar,int numParticles,int numMechTypes,std::vector< std::vector<double> >& workspace){
  stateList.clear(); // Make sure the stateList is empty
  if (modelNum == 1){
    // For model 1 (the fixed model), only ever sample the single valid state
    for (size_t i=0;i<numParticles;i++){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(0.0);
      x_state.params.push_back(0.0);
      stateList.push_back(x_state);
    }
  }
  else{
    // For any other model:
    // Determine shape of state space for this modelNum
    int numParams = MODEL_DESCRIPTIONS[modelNum][0]; // in globalVars.h
    int numVars = MODEL_DESCRIPTIONS[modelNum][1]; // in globalVars.h
    unsigned int size = numParams+numVars; // dimension of space
    std::vector<double> z (size,0.0); // holds standard normal variates

    // Get standard deviations
    double paramSD = sqrt(initParamVar);
    double varSD = sqrt(initVarVar);
    std::cout << paramSD << std::endl;
    std::cout << varSD << std::endl;
    // Sample numParticles particles, 
    // shape them into states, and place in vector
    size_t count = 0; // How many valid particles we have sampled
    size_t numRejected = 0; // How many particles we rejected
    bool reject;
    while (count<numParticles){

      // Sample size standard normal variates
      for (size_t i=0; i<size; i+=2){
	std::vector<double> variates = standardGaussianVariates();
	z[i] = variates[0];
	if (i != size-1) z[i+1] = variates[1];
      }
      // temporarily sample around the true mean
      // Create the state
      std::vector<double> mu;
      mu.push_back(-0.396);
      mu.push_back(-0.396);
      mu.push_back(0.56);      
      mu.push_back(0.7854);

      stateStruct x_state;
      x_state.model = modelNum;
      /*
      if (count<numParticles*.000001){
	for (size_t i=0; i<numParams; i++){
	  x_state.params.push_back(z[i]*.01+mu[i]); // replace .05 with paramSD
	}
	for (size_t i=numParams; i<size; i++){
	  x_state.vars.push_back(z[i]*.01+mu[i]); // replace .05 with varSD
	}
      }
      else{
	for (size_t i=0; i<numParams; i++){
	  x_state.params.push_back(z[i]*paramSD+mu[i]); // replace .05 with paramSD
	}
	for (size_t i=numParams; i<size; i++){
	  x_state.vars.push_back(z[i]*paramSD+mu[i]); // replace .05 with varSD
	}
      }
      */
      for (size_t i=0; i<numParams; i++){
	x_state.params.push_back(z[i]*paramSD*.1+mu[i]); // replace .05 with paramSD
      }
      for (size_t i=numParams; i<size; i++){
	x_state.vars.push_back(z[i]*paramSD*.1+mu[i]); // replace .05 with varSD
      }
      
      reject = false;
      
      /*
      if (translator::isStateValid(x_state,workspace)){
	stateList.push_back(x_state);
	count++;
	//if (count % 10000 == 9999) std::cout << count << std::endl;
      }
      */
      if (x_state.params[2]<=0){ /*std::cout << "r < 0" << std::endl;*/ reject=true;}
      else{
	// Check if state places rbt outside of rbt workspace
	double x = x_state.params[0]+x_state.params[2]*cos(x_state.vars[0]);
	double y = x_state.params[1]+x_state.params[2]*sin(x_state.vars[0]);
	//std::cout << x << "," << y << std::endl;
	if (x<workspace[0][0] || x>workspace[0][1] || 
	    y<workspace[1][0] || y>workspace[1][1]){ /*std::cout << "out of workspace" << std::endl;*/ reject=true;}
      }

      if (!reject){
	stateList.push_back(x_state);
	count++;
	//if (count % 10000 == 9999) std::cout << count << std::endl;
      }
      else numRejected++;
      
      //stateList.push_back(x_state);
      //count++;
    }
    std::cout << "Number Rejected: " << numRejected << std::endl;
  }
  /*
  // If you put this back, you have to 
  // add the right answer temporarily
  stateStruct rightState;
  rightState.model = 2;
  rightState.params.push_back(-0.396);
  rightState.params.push_back(-0.396);
  rightState.params.push_back(0.56);
  rightState.vars.push_back(0.7854);
  stateList.push_back(rightState);
  */

  // Set the probability of the samples equal across all models
  // prob per particle
  double probPerParticle = logUtils::safe_log(1.0/(numParticles*numMechTypes));
  logProbList.clear(); // Make sure the logProbList is empty
  std::vector<double> probs (numParticles,probPerParticle);
  logProbList = probs;
}

void setupUtils::setupParticlesRevSpecial(std::vector<stateStruct>& stateList,std::vector<double>& logProbList,int modelNum,double initParamVar,double initVarVar,int numParticles,int numMechTypes,std::vector< std::vector<double> >& workspace){
  stateList.clear(); // Make sure the stateList is empty
  if (modelNum == 1){
    // For model 1 (the fixed model), only ever sample the single valid state
    for (size_t i=0;i<numParticles;i++){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(0.0);
      x_state.params.push_back(0.0);
      stateList.push_back(x_state);
    }
  }
  else{
    // For any other model:
    // Determine shape of state space for this modelNum
    int numParams = MODEL_DESCRIPTIONS[modelNum][0]; // in globalVars.h
    int numVars = MODEL_DESCRIPTIONS[modelNum][1]; // in globalVars.h
    unsigned int size = numParams+numVars; // dimension of space
    std::vector<double> z (size,0.0); // holds standard normal variates

    // Get standard deviations
    double paramSD = sqrt(initParamVar);
    double varSD = sqrt(initVarVar);
    std::cout << paramSD << std::endl;
    std::cout << varSD << std::endl;
    // Sample numParticles particles, 
    // shape them into states, and place in vector
    size_t count = 0; // How many valid particles we have sampled
    size_t numRejected = 0; // How many particles we rejected
    bool reject;

    double pivotSD = 2.0;
    double handleSD = 0.01;

    while (count<numParticles){

      // Sample a pivot and handle by sampling standard normal variates
      std::vector<double> pivot = standardGaussianVariates();
      std::vector<double> handle = standardGaussianVariates();
      //handle[0] = 0.0;
      //handle[1] = 0.0;

      stateStruct x_state;
      x_state.model = modelNum;
      
      double xp = pivot[0]*pivotSD;
      double yp = pivot[1]*pivotSD;
      double xh = handle[0]*handleSD;
      double yh = handle[1]*handleSD;

      x_state.params.push_back(xp);
      x_state.params.push_back(yp);
      x_state.params.push_back(sqrt((xp-xh)*(xp-xh)+(yp-yh)*(yp-yh)));

      x_state.vars.push_back(atan2((yh-yp),(xh-xp)));
      
      reject = false;

      if (x_state.params[2]<=0){ /*std::cout << "r < 0" << std::endl;*/ reject=true;}
      else{
	// Check if state places rbt outside of rbt workspace
	if (xh<workspace[0][0] || xh>workspace[0][1] || 
	    yh<workspace[1][0] || yh>workspace[1][1]){ /*std::cout << "out of workspace" << std::endl;*/ reject=true;}
      }

      if (!reject){
	stateList.push_back(x_state);
	count++;
	//if (count % 10000 == 9999) std::cout << count << std::endl;
      }
      else numRejected++;
      
      //stateList.push_back(x_state);
      //count++;
    }
    std::cout << "Number Rejected: " << numRejected << std::endl;
  }
  /*
  // If you put this back, you have to 
  // add the right answer temporarily
  stateStruct rightState;
  rightState.model = 2;
  rightState.params.push_back(-0.396);
  rightState.params.push_back(-0.396);
  rightState.params.push_back(0.56);
  rightState.vars.push_back(0.7854);
  stateList.push_back(rightState);
  */

  // Set the probability of the samples equal across all models
  // prob per particle
  double probPerParticle = logUtils::safe_log(1.0/(numParticles*numMechTypes));
  logProbList.clear(); // Make sure the logProbList is empty
  std::vector<double> probs (numParticles,probPerParticle);
  logProbList = probs;
}

// This is the special sampling that we actually use
void setupUtils::setupParticlesSpecial(std::vector<stateStruct>& stateList,std::vector<double>& logProbList,int modelNum,double initParamVar,double initVarVar,int numParticles,int numMechTypes,std::vector< std::vector<double> >& workspace){
  stateList.clear(); // Make sure the stateList is empty
  if (modelNum == 0){
    // For model 0 (the free model), only ever sample the single valid state
    // which is vars x,y = (0,0)
    for (size_t i=0;i<numParticles;i++){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.vars.push_back(0.0);
      x_state.vars.push_back(0.0);
      stateList.push_back(x_state);
    }
  }
  else if (modelNum == 1){
    // For model 1 (the fixed model), only ever sample the single valid state
    // which is parameters x,y = (0,0)
    for (size_t i=0;i<numParticles;i++){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(0.0);
      x_state.params.push_back(0.0);
      stateList.push_back(x_state);
    }
  }
  else if (modelNum == 2){
    if(false){
      // For model 2 (the revolute model), only sample the pivot position and
      // calculate the other parameters and variables
      
      // Sample a pivot by sampling standard normal variates
      double pivotSD = 2.0;
      
      for (size_t i=0;i<numParticles;i++){
	std::vector<double> pivot = standardGaussianVariates();
	stateStruct x_state;
	x_state.model = modelNum;
	double xp = pivot[0]*pivotSD;
	double yp = pivot[1]*pivotSD;
	x_state.params.push_back(xp);
	x_state.params.push_back(yp);
	x_state.params.push_back(sqrt(xp*xp+yp*yp));
	x_state.vars.push_back(atan2(-yp,-xp));
	stateList.push_back(x_state);
      }
    }
    else{
      // For model 2 (the revolute model), sample a radius and an angle and 
      // calculate the other parameters
      
      // Sample a radius uniformly between min and max
      double rMin = 0.15;
      double rMax = 1.15;

      // Sample a angle uniformly between min and max
      double thMin = -M_PI+0.000000001; // maybe this is a good thing to do
      double thMax = M_PI;
      
      for (size_t i=0;i<numParticles;i++){
	double r = rMin+(rMax-rMin)*randomDouble();
	double th = thMin+(thMax-thMin)*randomDouble();

	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(r*cos(th)); // xp
	x_state.params.push_back(r*sin(th)); // yp
	x_state.params.push_back(r);
	if(th<=0) x_state.vars.push_back(th+M_PI);
	else x_state.vars.push_back(th-M_PI);
	stateList.push_back(x_state);
      }
    }
  }
  else if (modelNum == 3){
    // For model 3 (the prismatic model), only sample the angle and calculate
    // the other parameters and variables.
    // **** This model needs to be changed in the future because the pivot
    // is a redundant piece of information ****

    double offset = 0.40; // Pivot offset. This should be unnecessary.

    for (size_t i=0;i<numParticles;i++){
      // sample uniformly between -pi and pi
      double angle = 2*M_PI*((double)rand()/(double)RAND_MAX)-M_PI;
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(-offset*cos(angle));
      x_state.params.push_back(-offset*sin(angle));
      x_state.params.push_back(angle);
      x_state.vars.push_back(offset);
      stateList.push_back(x_state);
    }
  }
  else{
    throw std::invalid_argument("Sampler not setup for more than models 0-3.");
  }

  // Set the probability of the samples equal across all models
  // prob per particle
  double probPerParticle = logUtils::safe_log(1.0/(numParticles*numMechTypes));
  logProbList.clear(); // Make sure the logProbList is empty
  std::vector<double> probs (numParticles,probPerParticle);
  logProbList = probs;
}

// This is the special sampling that we actually use with repeating samples
void setupUtils::setupParticlesSpecialRepeat(std::vector<stateStruct>& stateList,std::vector<double>& logProbList,int modelNum,double initParamVar,double initVarVar,int numParticles,int numRepeats,int numMechTypes,std::vector< std::vector<double> >& workspace){
  stateList.clear(); // Make sure the stateList is empty
  if (modelNum == 0){
    // For model 0 (the free model), only ever sample the single valid state
    // which is vars x,y = (0,0)
    for (size_t i=0;i<numParticles;i+=numRepeats){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.vars.push_back(0.0);
      x_state.vars.push_back(0.0);
      for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
    }
  }
  else if (modelNum == 1){
    // For model 1 (the fixed model), only ever sample the single valid state
    // which is parameters x,y = (0,0)
    for (size_t i=0;i<numParticles;i+=numRepeats){
      stateStruct x_state;
      x_state.model = modelNum;
      x_state.params.push_back(0.0);
      x_state.params.push_back(0.0);
      for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
    }
  }
  else if (modelNum == 2){
    if(false){
      // For model 2 (the revolute model), only sample the pivot position and
      // calculate the other parameters and variables
      
      // Sample a pivot by sampling standard normal variates
      double pivotSD = 2.0;
      
      for (size_t i=0;i<numParticles;i+=numRepeats){
	std::vector<double> pivot = standardGaussianVariates();
	stateStruct x_state;
	x_state.model = modelNum;
	double xp = pivot[0]*pivotSD;
	double yp = pivot[1]*pivotSD;
	x_state.params.push_back(xp);
	x_state.params.push_back(yp);
	x_state.params.push_back(sqrt(xp*xp+yp*yp));
	x_state.vars.push_back(atan2(-yp,-xp));
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
    else if(false){
      // For model 2 (the revolute model), sample a radius and an angle and 
      // calculate the other parameters
      
      // Sample a radius uniformly between min and max
      //double rMin = 0.54;//0.15;
      //double rMax = 0.58;//1.15;
      double r = 0.56;

      // Sample an angle uniformly between min and max
      double thMin = -M_PI+0.000000001; // -2.5 // maybe this is a good thing to do
      double thMax = M_PI; // -2.1
      
      for (size_t i=0;i<numParticles;i+=numRepeats){
	//double r = rMin+(rMax-rMin)*randomDouble();
	double th = thMin+(thMax-thMin)*randomDouble();

	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(r*cos(th+M_PI)); // xp - flip angle
	x_state.params.push_back(r*sin(th+M_PI)); // yp - flip angle
	x_state.params.push_back(r);
	x_state.vars.push_back(th);
	//if(th<=0) x_state.vars.push_back(th+M_PI);
	//else x_state.vars.push_back(th-M_PI);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
    else if(false){
      // SAMPLE ONLY ONE SET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      // For model 2 (the revolute model), sample a radius and an angle and 
      // calculate the other parameters
      
      // Sample a radius uniformly between min and max
      //double rMin = 0.54;//0.15;
      //double rMax = 0.58;//1.15;
      double r = 0.56;

      // Sample an angle uniformly between min and max
      //double thMin = -M_PI+0.000000001; // -2.5 // maybe this is a good thing to do
      //double thMax = M_PI; // -2.1
      double th = -2.480;//-2.29;

      for (size_t i=0;i<numParticles;i+=numRepeats){
	//double r = rMin+(rMax-rMin)*randomDouble();
	//double th = thMin+(thMax-thMin)*randomDouble();

	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(r*cos(th+M_PI)); // xp - flip angle
	x_state.params.push_back(r*sin(th+M_PI)); // yp - flip angle
	x_state.params.push_back(r);
	x_state.vars.push_back(th);
	//if(th<=0) x_state.vars.push_back(th+M_PI);
	//else x_state.vars.push_back(th-M_PI);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
    else{
      // For model 2 (the revolute model), sample a radius and an angle and 
      // calculate the other parameters
      
      // Sample a radius uniformly between min and max
      double rMin = 0.15;
      double rMax = 1.15;

      // Sample an angle uniformly between min and max
      double thMin = -M_PI+0.000000001; // -2.5 // maybe this is a good thing to do
      double thMax = M_PI; // -2.1
      
      for (size_t i=0;i<numParticles;i+=numRepeats){
	double r = rMin+(rMax-rMin)*randomDouble();
	double th = thMin+(thMax-thMin)*randomDouble();

	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(r*cos(th+M_PI)); // xp - flip angle
	x_state.params.push_back(r*sin(th+M_PI)); // yp - flip angle
	x_state.params.push_back(r);
	x_state.vars.push_back(th);
	//if(th<=0) x_state.vars.push_back(th+M_PI);
	//else x_state.vars.push_back(th-M_PI);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
  }
  else if (modelNum == 3){
    // For model 3 (the prismatic model), only sample the angle and calculate
    // the other parameters and variables.
    // **** This model needs to be changed in the future because the pivot
    // is a redundant piece of information ****

    double offset = 0.40; // Pivot offset. This should be unnecessary.

    if(false){
      // SAMPLE ONLY ONE SET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      for (size_t i=0;i<numParticles;i+=numRepeats){
	// sample uniformly between -pi and pi
	//double angle = 2*M_PI*((double)rand()/(double)RAND_MAX)-M_PI;
	// sample uniformly between 0 and pi
	//double angle = M_PI*((double)rand()/(double)RAND_MAX);
	double angle = 2.232;//2.417;
	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(-offset*cos(angle));
	x_state.params.push_back(-offset*sin(angle));
	x_state.params.push_back(angle);
	x_state.vars.push_back(offset);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
    else{
      // NORMAL

      for (size_t i=0;i<numParticles;i+=numRepeats){
	// sample uniformly between -pi and pi
	//double angle = 2*M_PI*((double)rand()/(double)RAND_MAX)-M_PI;
	// sample uniformly between 0 and pi
	double angle = M_PI*((double)rand()/(double)RAND_MAX);
	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(-offset*cos(angle));
	x_state.params.push_back(-offset*sin(angle));
	x_state.params.push_back(angle);
	x_state.vars.push_back(offset);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
  }
  else if (modelNum == 4){
    // For model 4 (the latch 1 model), only sample the angle and calculate
    // the other parameters and variables.
    // **** This model needs to be changed in the future because the pivot
    // is a redundant piece of information ****


    if(false){
      // NORMAL

      for (size_t i=0;i<numParticles;i+=numRepeats){
	// sample uniformly between -pi and pi
	//double angle = 2*M_PI*((double)rand()/(double)RAND_MAX)-M_PI;
	// sample uniformly between 0 and pi
	double angle = M_PI*((double)rand()/(double)RAND_MAX);
	double offset = 0.30; // Pivot offset. This should be unnecessary.
	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(-offset*cos(angle));
	x_state.params.push_back(-offset*sin(angle));
	x_state.params.push_back(0.19);
	x_state.params.push_back(angle);
	x_state.params.push_back(0.11);

	x_state.vars.push_back(angle);
	x_state.vars.push_back(0.11);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
    else{
      // NORAML over two dimensions, th and d

      // set radius as constant
      double r = 0.19;

      // Sample a distance uniformly between min and max
      double dMin = 0.05;
      double dMax = 0.15;

      // Sample an angle uniformly between min and max
      double thMin = -M_PI+0.000000001; // maybe this is a good thing to do
      double thMax = M_PI;

      for (size_t i=0;i<numParticles;i+=numRepeats){
	// sample uniformly between -pi and pi
	double d = dMin+(dMax-dMin)*randomDouble();
	double th = thMin+(thMax-thMin)*randomDouble();
	double offset = r+d;
	stateStruct x_state;
	x_state.model = modelNum;
	x_state.params.push_back(-offset*cos(th));
	x_state.params.push_back(-offset*sin(th));
	x_state.params.push_back(r);
	x_state.params.push_back(th);
	x_state.params.push_back(d);

	x_state.vars.push_back(th);
	x_state.vars.push_back(d);
	for (size_t j=0;j<numRepeats;j++) stateList.push_back(x_state);
      }
    }
  }
  else{
    throw std::invalid_argument("Sampler not setup for more than models 0-3.");
  }

  // Set the probability of the samples equal across all models
  // prob per particle
  double probPerParticle = logUtils::safe_log(1.0/(numParticles*numMechTypes));
  logProbList.clear(); // Make sure the logProbList is empty
  std::vector<double> probs (numParticles,probPerParticle);
  logProbList = probs;
}


// END SAMPLING SECTION

////////////////////////////////////////////////////////////////////////////////
//                          End Particle Section                              //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                              Model Section                                 //
////////////////////////////////////////////////////////////////////////////////

//dimRanges (dimension ranges) gives you the min and max in each dimension.
//dimNums gives you the number of discrete points along a dimension.

std::vector<stateStruct> setupUtils::setupModel0(std::vector<stateStruct>& modelParamPairs){  
  // Model 0 is the free model
  // State looks like:
  // Model: 0
  // Params:
  // Vars: x,y in rbt space
  int modelNum = 0;
  int paramNum = 0; //how many parameters
  int varNum = 2; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0)); // empty size 0 vector  
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0); // empty size 0 vector
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -0.15;
  dRV[0][1] = 0.15;
  dRV[1][0] = -0.15;
  dRV[1][1] = 0.15;
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 11; // bs: 10
  dNV[1] = 11; // bs: 10

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel1(std::vector<stateStruct>& modelParamPairs){  
  // Model 1 is the fixed model
  // State looks like:
  // Model: 1
  // Params: x,y in rbt space
  // Vars: 
  int modelNum = 1;
  int paramNum = 2; //how many parameters
  int varNum = 0; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.0;
  dRP[0][1] = 0.0;
  dRP[1][0] = 0.0;
  dRP[1][1] = 0.0;
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0)); // empty size 0 vector
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0); // empty size 0 vector

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);

}

std::vector<stateStruct> setupUtils::setupModel2(std::vector<stateStruct>& modelParamPairs){  
  // Model 2 is the revolute model
  // State looks like:
  // Model: 2
  // Params: x_pivot,y_pivot in rbt space, r
  // Vars: theta in rbt space
  int modelNum = 2;
  int paramNum = 3; //how many parameters
  int varNum = 1; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[0][1] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[1][0] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[1][1] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[2][0] = 0.56; // ICRA 2014 - 0.3 - 0.44
  dRP[2][1] = 0.56; // ICRA 2014 - 0.3 - 0.44
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -3.14159; // bs: 3.14
  dRV[0][1] = 3.14159-0.065449846949787; // bs: 3.14
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 96; // bs: 100

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel3(std::vector<stateStruct>& modelParamPairs){  
  // Model 3 is the prismatic model
  // State looks like:
  // Model: 3
  // Params: x_axis,y_axis,theta_axis in rbt space
  // Vars: d
  int modelNum = 3;
  int paramNum = 3; //how many parameters
  int varNum = 1; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.0; // ICRA 2014 - -0.16
  dRP[0][1] = 0.0; // ICRA 2014 - -0.16
  dRP[1][0] = 0.226274; // ICRA 2014 - -0.16
  dRP[1][1] = 0.226274; // ICRA 2014 - -0.16
  dRP[2][0] = -1.570796; // ICRA 2014 - 0.7865 
  dRP[2][1] = -1.570796;  // ICRA 2014 - 0.7865
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -0.45255;
  dRV[0][1] = 0.45255;
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 49; // bs: 50

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel4(std::vector<stateStruct>& modelParamPairs){  
  // Model 4 is the revolute prismatic latch model
  // State looks like:
  // Model: 4
  // Params: x_pivot,y_pivot in rbt space, r, theta_L in rbt space, d_L
  // Vars: theta in rbt space, d
  int modelNum = 4;
  int paramNum = 5; //how many parameters
  int varNum = 2; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.27; // ICRA 2014 and Vid1 - -0.2
  dRP[0][1] = 0.27; // ICRA 2014 and Vid1 - -0.2
  dRP[1][0] = 0.0;
  dRP[1][1] = 0.0;
  dRP[2][0] = 0.17; // ICRA 2014 and Vid1 - 0.1
  dRP[2][1] = 0.17; // ICRA 2014 and Vid1 - 0.1
  dRP[3][0] = -3.14159; // ICRA 2014 and Vid1 - 0.0
  dRP[3][1] = -3.14159; // ICRA 2014 and Vid1 - 0.0
  dRP[4][0] = 0.1;
  dRP[4][1] = 0.1;
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  dNP[3] = 1;
  dNP[4] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -3.14159; // ICRA 2014 and Vid1 - -1.57
  dRV[0][1] = 3.14159-0.26179938779*.5; // bs: 3.14159-0.26179938779 // ICRA 2014 and Vid1 - 1.57
  dRV[1][0] = 0.0;
  dRV[1][1] = 0.30; // bs: 0.27 // ICRA 2014 and Vid1 - 0.2
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 24*2; // bs: 24 // ICRA 2014 and Vid1 - 12
  dNV[1] = 16; // bs: 10

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel5(std::vector<stateStruct>& modelParamPairs){  
  // Model 5 is the prismatic prismatic latch model
  // State looks like:
  // Model: 5
  // Params: x_axis2,y_axis2,theta_axis2 in rbt space, d_L2, d_L1
  // Vars: d_2, d_1
  int modelNum = 5;
  int paramNum = 5; //how many parameters
  int varNum = 2; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = -0.1;
  dRP[0][1] = -0.1;
  dRP[1][0] = -0.1;
  dRP[1][1] = -0.1;
  dRP[2][0] = 0.0;
  dRP[2][1] = 0.0;
  dRP[3][0] = 0.1;
  dRP[3][1] = 0.1;
  dRP[4][0] = 0.1;
  dRP[4][1] = 0.1;
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  dNP[3] = 1;
  dNP[4] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = 0.00; // used to be 0.01
  dRV[0][1] = 0.20;
  dRV[1][0] = 0.00; // used to be 0.01
  dRV[1][1] = 0.20;
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 9; // used to be 10
  dNV[1] = 9;

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

////////////////////////////////////////////////////////////////////////////////
//                             End Model Section                              //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                           Extra Model Section                              //
////////////////////////////////////////////////////////////////////////////////

std::vector<stateStruct> setupUtils::setupModel6(std::vector<stateStruct>& modelParamPairs){  
  // Model 2 is the revolute model
  // State looks like:
  // Model: 2
  // Params: x_pivot,y_pivot in rbt space, r
  // Vars: theta in rbt space
  int modelNum = 2;
  int paramNum = 3; //how many parameters
  int varNum = 1; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[0][1] = 0.396; // ICRA 2014 - 0.3 - 0.3111
  dRP[1][0] = -0.396; // ICRA 2014 - 0.3 - -0.3111
  dRP[1][1] = -0.396; // ICRA 2014 - 0.3 - -0.3111
  dRP[2][0] = 0.56; // ICRA 2014 - 0.3 - 0.44
  dRP[2][1] = 0.56; // ICRA 2014 - 0.3 - 0.44
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -3.14159; // bs: 3.14
  dRV[0][1] = 3.14159-0.065449846949787; // bs: 3.14
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 96; // bs: 100

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel7(std::vector<stateStruct>& modelParamPairs){  
  // Model 3 is the prismatic model
  // State looks like:
  // Model: 3
  // Params: x_axis,y_axis,theta_axis in rbt space
  // Vars: d
  int modelNum = 3;
  int paramNum = 3; //how many parameters
  int varNum = 1; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.226274; // bs: 0.16 // ICRA 2014 - -0.16
  dRP[0][1] = 0.226274; // bs: 0.16 // ICRA 2014 - -0.16
  dRP[1][0] = 0.0; // ICRA 2014 - -0.16
  dRP[1][1] = 0.0; // ICRA 2014 - -0.16
  dRP[2][0] = -3.14159; // ICRA 2014 - 0.7865 
  dRP[2][1] = -3.14159;  // ICRA 2014 - 0.7865
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -0.45255;
  dRV[0][1] = 0.45255;
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 49; // bs: 50

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel8(std::vector<stateStruct>& modelParamPairs){  
  // Model 4 is the revolute prismatic latch model
  // State looks like:
  // Model: 4
  // Params: x_pivot,y_pivot in rbt space, r, theta_L in rbt space, d_L
  // Vars: theta in rbt space, d
  int modelNum = 4;
  int paramNum = 5; //how many parameters
  int varNum = 2; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = -0.27; // ICRA 2014 and Vid1 - -0.2
  dRP[0][1] = -0.27; // ICRA 2014 and Vid1 - -0.2
  dRP[1][0] = 0.0;
  dRP[1][1] = 0.0;
  dRP[2][0] = 0.17; // ICRA 2014 and Vid1 - 0.1
  dRP[2][1] = 0.17; // ICRA 2014 and Vid1 - 0.1
  dRP[3][0] = 0.0; // ICRA 2014 and Vid1 - 0.0
  dRP[3][1] = 0.0; // ICRA 2014 and Vid1 - 0.0
  dRP[4][0] = 0.1;
  dRP[4][1] = 0.1;
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  dNP[3] = 1;
  dNP[4] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = -3.14159; // ICRA 2014 and Vid1 - -1.57
  dRV[0][1] = 3.14159-0.26179938779*.5; // bs: 3.14159-0.26179938779 // ICRA 2014 and Vid1 - 1.57
  dRV[1][0] = 0.0;
  dRV[1][1] = 0.30; // bs: 0.27 // ICRA 2014 and Vid1 - 0.2
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 24*2; // bs: 24 // ICRA 2014 and Vid1 - 12
  dNV[1] = 16; // bs: 10

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

std::vector<stateStruct> setupUtils::setupModel9(std::vector<stateStruct>& modelParamPairs){  
  // Model 5 is the prismatic prismatic latch model
  // State looks like:
  // Model: 5
  // Params: x_axis2,y_axis2,theta_axis2 in rbt space, d_L2, d_L1
  // Vars: d_2, d_1
  int modelNum = 5;
  int paramNum = 5; //how many parameters
  int varNum = 2; //how many variables

  //Dimension Ranges for Params
  std::vector< std::vector<double> > dRP (paramNum, std::vector<double> (2,0.0));
  dRP[0][0] = 0.1;
  dRP[0][1] = 0.1;
  dRP[1][0] = 0.1;
  dRP[1][1] = 0.1;
  dRP[2][0] = -3.14159;
  dRP[2][1] = -3.14159;
  dRP[3][0] = 0.1;
  dRP[3][1] = 0.1;
  dRP[4][0] = 0.1;
  dRP[4][1] = 0.1;
  //Dimension Numbers for Params
  std::vector<int> dNP (paramNum, 0);
  dNP[0] = 1;
  dNP[1] = 1;
  dNP[2] = 1;
  dNP[3] = 1;
  dNP[4] = 1;
  //Dimension Ranges for Vars
  std::vector< std::vector<double> > dRV (varNum, std::vector<double> (2,0.0));
  dRV[0][0] = 0.00; // used to be 0.01
  dRV[0][1] = 0.20;
  dRV[1][0] = 0.00; // used to be 0.01
  dRV[1][1] = 0.20;
  //Dimension Numbers for Vars
  std::vector<int> dNV (varNum, 0);
  dNV[0] = 9; // used to be 10
  dNV[1] = 9;

  return setupModelFromDec(dRP,dNP,dRV,dNV,modelNum,modelParamPairs);
}

////////////////////////////////////////////////////////////////////////////////
//                          End Extra Model Section                           //
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//                              Setup Section                                 //
////////////////////////////////////////////////////////////////////////////////

//Create the list of states
void setupUtils::setupStates(std::vector<stateStruct>& stateList,std::vector<stateStruct>& modelParamPairs){

  //setup model parameter pair lists for each model
  std::vector<stateStruct> modelParamPairs0;
  std::vector<stateStruct> modelParamPairs1;
  std::vector<stateStruct> modelParamPairs2;
  std::vector<stateStruct> modelParamPairs3;
  std::vector<stateStruct> modelParamPairs4;
  std::vector<stateStruct> modelParamPairs5;

  std::vector<stateStruct> modelParamPairs6;
  std::vector<stateStruct> modelParamPairs7;
  std::vector<stateStruct> modelParamPairs8;
  std::vector<stateStruct> modelParamPairs9;


  //Set up a state list for each model. 
  //Then stick together all of the lists into one master list. 

  std::vector<stateStruct> stateList0 = setupModel0(modelParamPairs0);
  std::vector<stateStruct> stateList1 = setupModel1(modelParamPairs1);
  std::vector<stateStruct> stateList2 = setupModel2(modelParamPairs2);
  std::vector<stateStruct> stateList3 = setupModel3(modelParamPairs3);
  std::vector<stateStruct> stateList4 = setupModel4(modelParamPairs4);
  std::vector<stateStruct> stateList5 = setupModel5(modelParamPairs5);

  std::vector<stateStruct> stateList6 = setupModel6(modelParamPairs6);
  std::vector<stateStruct> stateList7 = setupModel7(modelParamPairs7);
  std::vector<stateStruct> stateList8 = setupModel8(modelParamPairs8);
  std::vector<stateStruct> stateList9 = setupModel9(modelParamPairs9);


  //populate the modelParamPairs vector
  modelParamPairs = modelParamPairs0;
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs1.begin(), modelParamPairs1.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs2.begin(), modelParamPairs2.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs3.begin(), modelParamPairs3.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs4.begin(), modelParamPairs4.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs5.begin(), modelParamPairs5.end());

  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs6.begin(), modelParamPairs6.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs7.begin(), modelParamPairs7.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs8.begin(), modelParamPairs8.end());
  modelParamPairs.insert(modelParamPairs.end(), modelParamPairs9.begin(), modelParamPairs9.end());

  //populate the stateList vector
  stateList = stateList0;
  stateList.insert(stateList.end(), stateList1.begin(), stateList1.end());
  stateList.insert(stateList.end(), stateList2.begin(), stateList2.end());
  stateList.insert(stateList.end(), stateList3.begin(), stateList3.end());
  stateList.insert(stateList.end(), stateList4.begin(), stateList4.end());
  stateList.insert(stateList.end(), stateList5.begin(), stateList5.end());

  stateList.insert(stateList.end(), stateList6.begin(), stateList6.end());
  stateList.insert(stateList.end(), stateList7.begin(), stateList7.end());
  stateList.insert(stateList.end(), stateList8.begin(), stateList8.end());
  stateList.insert(stateList.end(), stateList9.begin(), stateList9.end());

}

void setupUtils::setupModelParamPairs(std::vector<stateStruct>& stateList,std::vector<stateStruct>& modelParamPairs,std::vector<int>& numVarTypesPerStateType){
  //1. Count up how many times certain instances occur
  //A state type is a model-parameter pair
  std::vector<stateStruct> tempModelParamPairs; //how many different model-parameter pairs
  numVarTypesPerStateType.clear(); //how many different variable sets per model-parameter pair
  bool addStateType;
  for (size_t i=0; i<stateList.size(); i++){
    addStateType = true;
    for (size_t j=0; j<tempModelParamPairs.size(); j++){
      if (stateList[i].model == tempModelParamPairs[j].model && stateList[i].params == tempModelParamPairs[j].params){
	addStateType = false;
	numVarTypesPerStateType[j]++;
	break; //the break assumes we didn't somehow add the same pair twice to the found list
      }
    }
    if (addStateType){
      stateStruct tempState = stateList[i];
      tempState.vars.clear(); // Remove variables from any model-param pair
      tempModelParamPairs.push_back(tempState);
      numVarTypesPerStateType.push_back(1);
    }
  }
  modelParamPairs.clear(); // Empty the vector before reassigning
  modelParamPairs = tempModelParamPairs;
}

//Overloaded
//setup a uniform prior over model and parameter pairs
void setupUtils::setupUniformPrior(std::vector<stateStruct>& stateList,std::vector<double>& probList){
  //1. Count up how many times certain instances occur
  //A state type is a model-parameter pair
  std::vector<stateStruct> foundStateTypes; //how many different model-parameter pairs
  std::vector<int> numVarTypesPerStateType; //how many different variable sets per model-parameter pair
  bool addStateType;
  for (size_t i=0; i<stateList.size(); i++){
    addStateType = true;
    for (size_t j=0; j<foundStateTypes.size(); j++){
      if (stateList[i].model == foundStateTypes[j].model && stateList[i].params == foundStateTypes[j].params){
	addStateType = false;
	numVarTypesPerStateType[j]++;
	break; //the break assumes we didn't somehow add the same pair twice to the found list
      }
    }
    if (addStateType){
      foundStateTypes.push_back(stateList[i]);
      numVarTypesPerStateType.push_back(1);
    }
  }

  //2. Figure out how much probability to assign to each instance
  double probPerStateType = (1.0/foundStateTypes.size());
  std::vector<double> probAmounts; //how much probability to assign per var type per state type. Same order as above.
  for (size_t i=0; i<foundStateTypes.size(); i++){
    probAmounts.push_back(probPerStateType/numVarTypesPerStateType[i]);
  }

  //3. Assing the probability to the right instances
  std::vector<double> expProbList; //exponatiated probabilities
  for (size_t i=0; i<stateList.size(); i++){
    for (size_t j=0; j<foundStateTypes.size(); j++){
      if (stateList[i].model == foundStateTypes[j].model && stateList[i].params == foundStateTypes[j].params){
	expProbList.push_back(probAmounts[j]);
      }	      
    }
  }

  //4. Convert probabilities to log form
  probList.clear(); //make sure this bad boy is empty
  for (size_t i = 0; i<expProbList.size(); i++){
    probList.push_back(logUtils::safe_log(expProbList[i]));
  }

}

// Overloaded - this one has the model-parameter pairs passed to it
// setup a uniform prior over model and parameter pairs
void setupUtils::setupUniformPrior(std::vector<stateStruct>& stateList,std::vector<double>& probList,std::vector<stateStruct>& modelParamPairs){
  // 1. Count up how many times certain instances occur
  // A state type is a model-parameter pair
  std::vector<int> numVarTypesPerStateType (modelParamPairs.size(),0); //how many different variable sets per model-parameter pair
  for (size_t i=0; i<stateList.size(); i++){
    for (size_t j=0; j<modelParamPairs.size(); j++){
      if (stateList[i].model == modelParamPairs[j].model && stateList[i].params == modelParamPairs[j].params){
	numVarTypesPerStateType[j]++;
	break; //the break assumes we didn't somehow add the same pair twice to the found list
      }
    }
  }

  // 2. Figure out how much probability to assign to each instance
  double probPerStateType = (1.0/modelParamPairs.size());
  std::vector<double> probAmounts; //how much probability to assign per var type per state type. Same order as above.
  for (size_t i=0; i<modelParamPairs.size(); i++){
    probAmounts.push_back(probPerStateType/numVarTypesPerStateType[i]);
  }

  // 3. Assing the probability to the right instances
  std::vector<double> expProbList; //exponatiated probabilities
  for (size_t i=0; i<stateList.size(); i++){
    for (size_t j=0; j<modelParamPairs.size(); j++){
      if (stateList[i].model == modelParamPairs[j].model && stateList[i].params == modelParamPairs[j].params){
	expProbList.push_back(probAmounts[j]);
      }	      
    }
  }

  // 4. Convert probabilities to log form
  probList.clear(); // make sure this bad boy is empty
  for (size_t i = 0; i<expProbList.size(); i++){
    probList.push_back(logUtils::safe_log(expProbList[i]));
  }

}

// This was here when you only had one model. You updated it on 2/5/14
/*
// setup a gaussian prior over cartesian positions of states
void setupUtils::setupGaussianPrior(std::vector<stateStruct>& stateList,std::vector<double>& probList){

  probList.clear(); // make sure this bad boy is empty

  // holders
  std::vector<double> tempCartPosInRbt;
  std::vector<double> assumedCartStartPosInRbt (2,0.0); // this is just your basic assumption. that the robot starts it's gripper at 0,0.

  // Define covariance matrices
  // Might want to change this later so that it's the same as something that makes sense already
  // set additional variables
  // create inverse matrix (hard coded)
  //double invObsArray[] = {100.0,0.0,0.0,100.0}; // change
  double invObsArray[] = {10000.0,0.0,0.0,10000.0};

  std::vector<double> invObsCovMat;
  invObsCovMat.assign(invObsArray, invObsArray + sizeof(invObsArray)/sizeof(double));
  
  // create determinant (hard coded)
  //double detCovMat = 0.0001; // change
  double detCovMat = 0.00000001;

  for (size_t i=0;i<stateList.size();i++){
    tempCartPosInRbt = translator::translateStToRbt(stateList[i]);
    probList.push_back(logUtils::evaluteLogMVG(tempCartPosInRbt,assumedCartStartPosInRbt,invObsCovMat,detCovMat));
  }

  // probList is not normalized because of the discretization
  // normalize
  probList = logUtils::normalizeVectorInLogSpace(probList);

}
*/

// Overloaded
// setup a gaussian prior over cartesian positions of states
void setupUtils::setupGaussianPrior(std::vector<stateStruct>& stateList,std::vector<double>& probList){
  // 0. Setup the Gaussian used to calculate the probabilities of each state.
  probList.clear(); // make sure this bad boy is empty

  // holders
  std::vector<double> tempCartPosInRbt;
  std::vector<double> assumedCartStartPosInRbt (2,0.0); // this is just your basic assumption. that the robot starts it's gripper at 0,0.

  // Define covariance matrices
  // Might want to change this later so that it's the same as something that makes sense already
  // set additional variables
  // create inverse matrix (hard coded)
  //double invObsArray[] = {100.0,0.0,0.0,100.0}; // change
  double invObsArray[] = {10000.0,0.0,0.0,10000.0};

  std::vector<double> invObsCovMat;
  invObsCovMat.assign(invObsArray, invObsArray + sizeof(invObsArray)/sizeof(double));
  
  // create determinant (hard coded)
  //double detCovMat = 0.0001; // change
  double detCovMat = 0.00000001;

  // 1. Count up how many times certain instances occur
  // A state type is a model-parameter pair
  std::vector<stateStruct> foundStateTypes; // how many different model-parameter pairs
  std::vector<int> numVarTypesPerStateType; // how many different variable sets per model-parameter pair
  std::vector<std::vector<double> > stateTypeProbLists; // a list of lists of probabilites of the foundStateTypes
  bool addStateType;
  for (size_t i=0; i<stateList.size(); i++){
    addStateType = true;
    for (size_t j=0; j<foundStateTypes.size(); j++){
      if (stateList[i].model == foundStateTypes[j].model && stateList[i].params == foundStateTypes[j].params){
	addStateType = false;
	numVarTypesPerStateType[j]++;
	// Calculate value from Gaussian distribution
	tempCartPosInRbt = translator::translateStToRbt(stateList[i]);
	stateTypeProbLists[j].push_back(logUtils::evaluteLogMVG(tempCartPosInRbt,assumedCartStartPosInRbt,invObsCovMat,detCovMat));
	break; //the break assumes we didn't somehow add the same pair twice to the found list
      }
    }
    if (addStateType){
      foundStateTypes.push_back(stateList[i]);
      numVarTypesPerStateType.push_back(1);
      // Calculate value from Gaussian distribution
      tempCartPosInRbt = translator::translateStToRbt(stateList[i]);
      std::vector<double> tempProbVect;
      tempProbVect.push_back(logUtils::evaluteLogMVG(tempCartPosInRbt,assumedCartStartPosInRbt,invObsCovMat,detCovMat));
      stateTypeProbLists.push_back(tempProbVect);
    }
  }

  // 2. Normalize the vectors in stateTypeProbLists.
  for (size_t i=0; i<stateTypeProbLists.size(); i++){
    stateTypeProbLists[i] = logUtils::normalizeVectorInLogSpace(stateTypeProbLists[i]);
  }

  // 3. Figure out how much probability to assign to each model-param pair
  double probPerStateType = logUtils::safe_log((1.0/foundStateTypes.size()));

  /*
  std::cout << "hi" << std::endl;
  for (size_t i=0; i<stateTypeProbLists.size(); i++){
    std::cout << i << std::endl;
    for (size_t j=0; j<stateTypeProbLists[i].size(); j++){
      std::cout << stateTypeProbLists[i][j] << ",";
    }
    std::cout << std::endl;
  }
  */

  // 4. Assign the probabilities to probList in the correct order.
  // Also scale the probabilities (by adding) so the final distribution is a distribution
  // (this assumes traversing stateList happens the same way every time. Which is probably true)
  // This is FIFO
  for (size_t i=0; i<stateList.size(); i++){
    //std::cout << i << std::endl;
    for (size_t j=0; j<foundStateTypes.size(); j++){
      if (stateList[i].model == foundStateTypes[j].model && stateList[i].params == foundStateTypes[j].params){
	/*
	std::cout << "first" << std::endl;
	std::cout << stateTypeProbLists[j][0] << std::endl;
	std::cout << j << std::endl;
	std::cout << probPerStateType << std::endl;
	*/
	probList.push_back(stateTypeProbLists[j][0]+probPerStateType); // access the first probability for that model parameter type + scale probabilities
	//std::cout << "second" << std::endl;
	stateTypeProbLists[j].erase(stateTypeProbLists[j].begin()); // erase the first element
	break; // once you find the right type for a state, no need to keep iterating
      }
    }
  }

}

// Overloaded - this one has the model-parameter pairs passed to it
// setup a gaussian prior over cartesian positions of states
void setupUtils::setupGaussianPrior(std::vector<stateStruct>& stateList,std::vector<double>& probList,std::vector<stateStruct>& modelParamPairs){
  // 0. Setup the Gaussian used to calculate the probabilities of each state.
  probList.clear(); // make sure this bad boy is empty

  // holders
  std::vector<double> tempCartPosInRbt;
  std::vector<double> assumedCartStartPosInRbt (2,0.0); // this is just your basic assumption. that the robot starts it's gripper at 0,0.

  // Define covariance matrices
  // Might want to change this later so that it's the same as something that makes sense already
  // set additional variables
  // create inverse matrix (hard coded)
  //double invObsArray[] = {100.0,0.0,0.0,100.0}; // change
  double invObsArray[] = {10000.0,0.0,0.0,10000.0};

  std::vector<double> invObsCovMat;
  invObsCovMat.assign(invObsArray, invObsArray + sizeof(invObsArray)/sizeof(double));
  
  // create determinant (hard coded)
  //double detCovMat = 0.0001; // change
  double detCovMat = 0.00000001;

  // 1. Count up how many times certain instances occur
  // A state type is a model-parameter pair
  std::vector<int> numVarTypesPerStateType (modelParamPairs.size(),0); // how many different variable sets per model-parameter pair
  std::vector<std::vector<double> > stateTypeProbLists (modelParamPairs.size(),std::vector<double>() ); // a list of lists of probabilites of the modelParamPairs
  for (size_t i=0; i<stateList.size(); i++){
    for (size_t j=0; j<modelParamPairs.size(); j++){
      if (stateList[i].model == modelParamPairs[j].model && stateList[i].params == modelParamPairs[j].params){
	numVarTypesPerStateType[j]++;
	// Calculate value from Gaussian distribution
	tempCartPosInRbt = translator::translateStToRbt(stateList[i]);
	stateTypeProbLists[j].push_back(logUtils::evaluteLogMVG(tempCartPosInRbt,assumedCartStartPosInRbt,invObsCovMat,detCovMat));
	break; //the break assumes we didn't somehow add the same pair twice to the found list
      }
    }
  }

  // 2. Normalize the vectors in stateTypeProbLists.
  for (size_t i=0; i<stateTypeProbLists.size(); i++){
    stateTypeProbLists[i] = logUtils::normalizeVectorInLogSpace(stateTypeProbLists[i]);
  }

  // 3. Figure out how much probability to assign to each model-param pair
  double probPerStateType = logUtils::safe_log((1.0/modelParamPairs.size()));

  /*
  std::cout << "hi" << std::endl;
  for (size_t i=0; i<stateTypeProbLists.size(); i++){
    std::cout << i << std::endl;
    for (size_t j=0; j<stateTypeProbLists[i].size(); j++){
      std::cout << stateTypeProbLists[i][j] << ",";
    }
    std::cout << std::endl;
  }
  */

  // 4. Assign the probabilities to probList in the correct order.
  // Also scale the probabilities (by adding) so the final distribution is a distribution
  // (this assumes traversing stateList happens the same way every time. Which is probably true)
  // This is FIFO
  for (size_t i=0; i<stateList.size(); i++){
    //std::cout << i << std::endl;
    for (size_t j=0; j<modelParamPairs.size(); j++){
      if (stateList[i].model == modelParamPairs[j].model && stateList[i].params == modelParamPairs[j].params){
	/*
	std::cout << "first" << std::endl;
	std::cout << stateTypeProbLists[j][0] << std::endl;
	std::cout << j << std::endl;
	std::cout << probPerStateType << std::endl;
	*/
	probList.push_back(stateTypeProbLists[j][0]+probPerStateType); // access the first probability for that model parameter type + scale probabilities
	//std::cout << "second" << std::endl;
	stateTypeProbLists[j].erase(stateTypeProbLists[j].begin()); // erase the first element
	break; // once you find the right type for a state, no need to keep iterating
      }
    }
  }

}


//Create the list of actions
void setupUtils::setupActions(std::vector< std::vector<double> >& actionList){
  /*
  std::vector<double> action1;
  std::vector<double> action2;
  std::vector<double> action3;
  std::vector<double> action4;

  action1.push_back(0.06);
  action1.push_back(0.06);

  action2.push_back(0.06);
  action2.push_back(-0.06);

  action3.push_back(-0.06);
  action3.push_back(-0.06);

  action4.push_back(-0.06);
  action4.push_back(0.06);

  actionList.clear();
  actionList.push_back(action1);
  actionList.push_back(action2);
  actionList.push_back(action3);
  actionList.push_back(action4);
  */
  
  if(RELATIVE){
    /*
    std::vector<double> action1;
    std::vector<double> action2;
    std::vector<double> action3;
    std::vector<double> action4;
    
    action1.push_back(0.06);
    action1.push_back(0.06);
    
    action2.push_back(0.06);
    action2.push_back(-0.06);
    
    action3.push_back(-0.06);
    action3.push_back(-0.06);
    
    action4.push_back(-0.06);
    action4.push_back(0.06);
    
    actionList.clear();
    actionList.push_back(action1);
    actionList.push_back(action2);
    actionList.push_back(action3);
    actionList.push_back(action4);
    */
    
    //This is totally 2D
    // In a circle around the gripper
    int numPts = 8; // how many points around the circle
    double radius = 0.12; // radius of the points //used to be .06, big is .12
    double angleDelta = 2*M_PI/numPts;

    actionList.clear(); // clear anything in there
    std::vector<double> tempAction; // slightly faster if outside

    for (size_t i=0;i<numPts;i++){
      tempAction.clear();
      tempAction.push_back(radius*cos(i*angleDelta)); // add x component
      tempAction.push_back(radius*sin(i*angleDelta)); // add y component
      actionList.push_back(tempAction);
    }
    for(size_t i=0;i<actionList.size();i++){
      std::cout << actionList[i][0] << "," << actionList[i][1] << std::endl;
    }
    
  }
  else{
    /*
    std::vector<double> action1;
    std::vector<double> action2;
    std::vector<double> action3;
    std::vector<double> action4;
    
    action1.push_back(0.06);
    action1.push_back(0.06);
    
    action2.push_back(0.12);
    action2.push_back(0.0);
    
    action3.push_back(0.06);
    action3.push_back(-0.06);
    
    action4.push_back(0.0);
    action4.push_back(0.0);
    
    actionList.clear();
    actionList.push_back(action1);
    actionList.push_back(action2);
    actionList.push_back(action3);
    actionList.push_back(action4);
    */

    // how many dimensions for an action
    int actDimNum = 2;
    
    //Dimension Ranges for Actions
    std::vector< std::vector<double> > dRA (actDimNum, std::vector<double> (2,0.0));
    dRA[0][0] = -0.12;
    dRA[0][1] = 0.12;
    dRA[1][0] = -0.12;
    dRA[1][1] = 0.12;
    //Dimension Numbers for Actions
    std::vector<int> dNA (actDimNum, 0);
    dNA[0] = 3;
    dNA[1] = 3;
    
    //setup for Actions
    actionList = dimsToList(dRA,dNA);
    
  }
  
}

////////////////////////////////////////////////////////////////////////////////
//                             End Setup Section                              //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                             Validate Section                               //
////////////////////////////////////////////////////////////////////////////////

void setupUtils::validateStates(std::vector<stateStruct>& stateList,std::vector< std::vector<double> >& workspace){
  // All workspace stuff right now assumes a 2D workspace - FIX
  // States must be within the workspace and satisfy model-specific conditions
  std::vector<stateStruct> tempStates;
  for (size_t i=0;i<stateList.size();i++){
    if (translator::isStateValid(stateList[i],workspace)){
      tempStates.push_back(stateList[i]);
    }
  }
  stateList.clear();
  stateList = tempStates;
}

void setupUtils::validateActions(std::vector< std::vector<double> >& actionList,std::vector< std::vector<double> >& workspace){
  // All workspace stuff right now assumes a 2D workspace - FIX
  // Actions must be within the workspace
  std::vector< std::vector<double> > tempActions;
  for (size_t i=0;i<actionList.size();i++){
    if (!(actionList[i][0]<workspace[0][0] || actionList[i][0]>workspace[0][1] || actionList[i][1]<workspace[1][0] || actionList[i][1]>workspace[1][1])){
      tempActions.push_back(actionList[i]);
    }
  }
  actionList.clear();
  actionList = tempActions;
}

////////////////////////////////////////////////////////////////////////////////
//                           End Validate Section                             //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                               Aux Section                                  //
////////////////////////////////////////////////////////////////////////////////

std::vector<stateStruct> setupUtils::setupModelFromDec(std::vector< std::vector<double> >& dRP,std::vector<int>& dNP,std::vector< std::vector<double> >& dRV,std::vector<int>& dNV,int& modelNum,std::vector<stateStruct>& modelParamPairs){
  //setup for model from decleration in setupModel*
  std::vector< std::vector<double> > paramsList = dimsToList(dRP,dNP);
  std::vector< std::vector<double> > varsList = dimsToList(dRV,dNV);

  //fill in model-parameter pairs. vars is blank for a stateType
  modelParamPairs.clear();
  stateStruct tempState;
  tempState.model=modelNum;

  for (size_t i=0;i<paramsList.size();i++){
    tempState.params=paramsList[i];
    modelParamPairs.push_back(tempState);
  }

  return createStateListForModel(modelNum,paramsList,varsList);
}

std::vector<stateStruct> setupUtils::createStateListForModel(int modelNum,std::vector< std::vector<double> > paramsList,std::vector< std::vector<double> > varsList){
  std::vector<stateStruct> stateList;
  stateStruct tempState;
  tempState.model=modelNum;
  for (size_t i=0; i<paramsList.size(); i++){
    tempState.params=paramsList[i];
    for (size_t j=0; j<varsList.size(); j++){
      tempState.vars=varsList[j];
      stateList.push_back(tempState);
    }
  }
  return stateList;
}

//convert from set of dimension ranges and number of points to a list of points
std::vector< std::vector<double> > setupUtils::dimsToList(std::vector< std::vector<double> > dimRanges, std::vector<int> dimNums){
  std::vector< std::vector<double> > valueList = createValueList(dimRanges,dimNums);
  return recurseList(std::vector< std::vector<double> > (), std::vector<double> (), 0, valueList);
}

//create the value list to set up states
std::vector< std::vector<double> > setupUtils::createValueList(std::vector< std::vector<double> > dimRanges, std::vector<int> dimNums){
  //there might be a shorter way 
  std::vector< std::vector<double> > valueList;
  int dims = dimNums.size(); //how many dimension numbers is the number of dimensions
  for (size_t i=0; i<dims; i++) {
    double delta = 0.0;
    if (dimNums[i]>1){
      delta = (dimRanges[i][1]-dimRanges[i][0])/(dimNums[i]-1); //-1 because of the spacing
    }
    std::vector<double> tempVect;
    for (size_t j=0; j<dimNums[i]; j++) {
      // This is ridiculous - huge SAS problem - crazy fix
      // HACK THIS IS NOT A FIX DELETE CHECK
      tempVect.push_back(dimRanges[i][0]+delta*j);
    }
    valueList.push_back(tempVect);
  }
  return valueList;
}


//the recursive function
std::vector< std::vector<double> > setupUtils::recurseList(std::vector< std::vector<double> > totalList, std::vector<double> oldSeq, int level, std::vector< std::vector<double> > valueList){
  //if (level>valueList.size()-1) {
  if (level>=valueList.size()) {
    totalList.push_back(oldSeq);
  }
  else {
    for (size_t i=0; i<valueList[level].size(); i++) {
      oldSeq.push_back(valueList[level][i]);
      totalList = recurseList(totalList,oldSeq,level+1,valueList);
      oldSeq.pop_back();
    }
  }
  return totalList;
}

////////////////////////////////////////////////////////////////////////////////
//                             End Aux Section                                //
////////////////////////////////////////////////////////////////////////////////
