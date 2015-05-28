#include "translator.h"
#include "logUtils.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iterator>

// include new mechanism versions
#include "latch1.h"

#include <iostream> // DELETE

// overloaded
stateStruct translator::stateTransition(stateStruct& state, std::vector<double>& action){
  // Go back to this
  /* 
  Mechanism* mechPtr = createMechanism(state.model);
  stateStruct nextState = mechPtr->initAndSim(state,action);
  delete mechPtr;
  return nextState;
  */
  // Just for now to speed things up
  stateStruct nextState = state;

  if (state.model == 0){
    // Perfect relative motion from current location
    nextState.vars[0] += action[0];
    nextState.vars[1] += action[1];
  }
  else if (state.model == 1){
    // Do nothing
  }
  else if (state.model == 2){
    // Old Way
    // Calculate equilibrium point

    // this used to be after
    /*
      for (size_t j=0;j<state.params.size();j++){
      state.params[j] 
      //+= 0.01*(2*(actionSelection::randomDouble()-0.5));
      += RealWorld::gaussianNoise();
      }
    */

    if(false){
      double x = action[0]+state.params[2]*cos(state.vars[0]);
      double y = action[1]+state.params[2]*sin(state.vars[0]);
      nextState.vars[0] = atan2(y,x);
    }
    else{
      // new way
      double thi = state.vars[0];
      int numSteps = 360;
      double dth = 2*M_PI/numSteps;
      std::vector<double> E(numSteps);
      double r = state.params[2];
      double ax = action[0];
      double ay = action[1];
      double KxP = 100;
      double KyP = 400;

      for(size_t i=0;i<numSteps;i++){
	double th = -M_PI+(i+1)*dth;
	E[i]=0.5*KxP*pow((r*((cos(thi)-cos(th))*ay-(sin(thi)-sin(th))*ax)),2)
	  + 0.5*KyP*pow((r*((cos(thi)-cos(th))*ax+(sin(thi)-sin(th))*ay)
			 +ax*ax+ay*ay),2);
      }
      nextState.vars[0] = -M_PI
	+(std::distance(E.begin(),std::min_element(E.begin(),E.end()))+1)*dth;
    }
  }
  else if (state.model == 3){
    // Calculate equilibrium point
    nextState.vars[0] += action[0]*cos(state.params[2])
      +action[1]*sin(state.params[2]);
  }
  else if (state.model == 4){
    latch1::simulate(nextState.params,nextState.vars,action);
  }	

  return nextState;
}

std::vector<double> translator::translateStToObs(stateStruct& state){
  if (state.model == 0){
    // State looks like:
    // Model: 0
    // Params:
    // Vars: x,y in rbt space

    // Observation looks like:
    // x,y in rbt space
    return state.vars;
  }
  else if (state.model == 1){
    // State looks like:
    // Model: 1
    // Params: x,y in rbt space
    // Vars: 

    // Observation looks like:
    // x,y in rbt space
    return state.params;
  }
  else if (state.model == 2){
    // State looks like:
    // Model: 2
    // Params: x_pivot,y_pivot in rbt space, r
    // Vars: theta in rbt space  
    double theta = state.vars[0];
    double x_p = state.params[0];
    double y_p = state.params[1];
    double r = state.params[2];
  
    // Observation looks like:
    // x,y in rbt space
    std::vector<double> obs (2,0.0);

    // Calculate
    obs[0] = x_p+r*cos(theta); // set x_obs
    obs[1] = y_p+r*sin(theta); // set y_obs
    return obs;
  }
  else if (state.model == 3){
    // State looks like:
    // Model: 3
    // Params: x_axis,y_axis,theta_axis in rbt space
    // Vars: d
    
    double d = state.vars[0];
    double x_a = state.params[0];
    double y_a = state.params[1];
    double theta_a = state.params[2];
    
    // Observation looks like:
    // x,y in rbt space
    std::vector<double> obs (2,0.0);
    
    // Calculate
    obs[0] = x_a+d*cos(theta_a); // set x_obs
    obs[1] = y_a+d*sin(theta_a); // set y_obs
    return obs;
  }
  else if (state.model == 4){
    // State looks like:
    // Model: 4
    // Params: x_pivot,y_pivot in rbt space, r, theta_L in rbt space, d_L
    // Vars: theta in rbt space, d

    double theta = state.vars[0];
    double d = state.vars[1];
    double x_p = state.params[0];
    double y_p = state.params[1];
    double r = state.params[2];
  
    // Observation looks like:
    // x,y in rbt space
    std::vector<double> obs (2,0.0);

    // Calculate
    obs[0] = x_p+(r+d)*cos(theta); // set x_obs
    obs[1] = y_p+(r+d)*sin(theta); // set y_obs
    return obs;
  }  

}

bool translator::isStateValid(stateStruct& state,
			      std::vector< std::vector<double> >& workspace){
  if (state.model == 0){
    // State looks like:
    // Model: 0
    // Params:
    // Vars: x,y in rbt space
  
    // Single conditions to check
    // Check if state places rbt outside of rbt workspace
    std::vector<double> rbt = translateStToObs(state);
    if (rbt[0]<workspace[0][0] || rbt[0]>workspace[0][1] 
	|| rbt[1]<workspace[1][0] || rbt[1]>workspace[1][1]){ 
      /*std::cout << "error 2" << std::endl;*/ 
      return false;
    }
    else return true;
  }
  else if (state.model == 1){
    // State looks like:
    // Model: 1
    // Params: x,y in rbt space
    // Vars: 
    
    // Single conditions to check
    // Check if state places rbt outside of rbt workspace
    std::vector<double> rbt = translateStToObs(state);
    if (rbt[0]<workspace[0][0] || rbt[0]>workspace[0][1] 
	|| rbt[1]<workspace[1][0] || rbt[1]>workspace[1][1]){ 
      /*std::cout << "error 2" << std::endl;*/ 
      return false;
    }
    else return true;
  }
  else if (state.model == 2){
    // State looks like:
    // Model: 2
    // Params: x_pivot,y_pivot in rbt space, r
    // Vars: theta in rbt space
  
    // Multiple conditions to check in order of increasing cost of computation
    double r = state.params[2];
    // Check if state violates the basic rules of this latch
    if (r<=0){ /*std::cout << "error 1" << std::endl;*/ return false;}
    else{
      // Check if state places rbt outside of rbt workspace
      std::vector<double> rbt = translateStToObs(state);
      if (rbt[0]<workspace[0][0] || rbt[0]>workspace[0][1] 
	  || rbt[1]<workspace[1][0] || rbt[1]>workspace[1][1]){ 
	/*std::cout << "error 2" << std::endl;*/ 
	return false;
      }
      else return true;
    }
  }
  else if (state.model == 3){
    // State looks like:
    // Model: 3
    // Params: x_axis,y_axis,theta_axis in rbt space
    // Vars: d
    
    // Single conditions to check
    // Check if state places rbt outside of rbt workspace
    std::vector<double> rbt = translateStToObs(state);
    if (rbt[0]<workspace[0][0] || rbt[0]>workspace[0][1] 
	|| rbt[1]<workspace[1][0] || rbt[1]>workspace[1][1]){ 
      /*std::cout << "error 2" << std::endl;*/ 
      return false;}
    else return true;
  }
  else if (state.model == 4){
    // State looks like:
    // Model: 4
    // Params: x_pivot,y_pivot in rbt space, r, theta_L in rbt space, d_L
    // Vars: theta in rbt space, d
    
    // Multiple conditions to check in order of increasing cost of computation
    double d = state.vars[1];
    double r = state.params[2];
    double d_L = state.params[4];
    // Check if state violates the basic rules of this latch
    if (d<0 || d>r || r<d_L || r<=0 || d_L<=0){ /*std::cout << "error 1" << std::endl;*/ return false;}
    else{
      // Check if state places rbt outside of rbt workspace
      std::vector<double> rbt = translateStToObs(state);
      if (rbt[0]<workspace[0][0] || rbt[0]>workspace[0][1] || rbt[1]<workspace[1][0] || rbt[1]>workspace[1][1]){ /*std::cout << "error 2" << std::endl;*/ return false;}
      // HERE YOU SHOULD CHECK FOR COLLISIONS
    }
  } 
}
  
