#include <iostream> // cout, endl
#define _USE_MATH_DEFINES 
#include <cmath>
#include "auxUtils.h"

// Check if two vectors are nearly colinear
bool auxUtils::nearlyColinear(std::vector<double> &a, std::vector<double> &b,
			      double &threshAngle){
  // Calculate the square of the cosine of the threshold angle
  double sqCosThresh = cos(threshAngle)*cos(threshAngle);
  // Calculate the square of the consine of the angle between a and b
  double sqCos = (Vdot(a,b)*Vdot(a,b))/(VsqMag(a)*VsqMag(b));
  return sqCos>=sqCosThresh;
}

// Subtract two vectors
std::vector<double> auxUtils::Vminus(std::vector<double> &a, 
				     std::vector<double> &b){
  std::vector<double> ans (2,0.0);
  ans[0] = a[0]-b[0];
  ans[1] = a[1]-b[1];
  return ans;
}

// Return the cross product of two vectors
double auxUtils::Vcross(std::vector<double> &a, std::vector<double> &b){
  return a[0]*b[1]-a[1]*b[0];
}

// Return the dot product of two vectors
double auxUtils::Vdot(std::vector<double> &a, std::vector<double> &b){
  return a[0]*b[0]+a[1]*b[1];
}

// Return the square magnitude of the vector
double auxUtils::VsqMag(std::vector<double> &a){
  return a[0]*a[0]+a[1]*a[1];
}

void auxUtils::printState(stateStruct &state){
  std::cout << "State:" << std::endl;
  std::cout << "Model: " << state.model << std::endl;
  std::cout << "Params: ";
  for (size_t j = 0; j<state.params.size(); j++) {
    std::cout << state.params[j] << ',';
  }
  std::cout << std::endl;
  std::cout << "Vars: ";
  for (size_t j = 0; j<state.vars.size(); j++) {
    std::cout << state.vars[j] << ',';
  }
  std::cout << std::endl;
}

void auxUtils::printAction(std::vector<double> &action){
  std::cout << "Action:" << std::endl;
  for (size_t j = 0; j<action.size(); j++) {
    std::cout << action[j] << ',';
  }
  std::cout << std::endl;
}

/*
// g++ auxUtils.cpp -o auxUtilsTest.o
int main(){
  double th = -M_PI/4;
  double threshAngle = M_PI/4;
  std::vector<double> a (2,0.0);
  a[0] = 2.0;
  a[1] = 2.0;
  std::vector<double> b (2,0.0);
  b[0] = cos(th);
  b[1] = sin(th);
  std::cout << a[0] << "," << a[1] << std::endl;
  std::cout << b[0] << "," << b[1] << std::endl;
  std::cout << auxUtils::nearlyColinear(a,b,threshAngle) << std::endl;

  
  return 1;
}
*/
