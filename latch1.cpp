// http://www.blackpawn.com/texts/pointinpoly/
#include "latch1.h"
#include "logUtils.h"

#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES 
//#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h> // rand

const double _L = 0.05;
const double _W = 0.038;


////////////////////////////////////////////////////////////////////////////////
// Aux
////////////////////////////////////////////////////////////////////////////////

std::vector<double> latch1::Vminus(std::vector<double> &a, 
				   std::vector<double> &b){
  std::vector<double> ans (2,0.0);
  ans[0] = a[0]-b[0];
  ans[1] = a[1]-b[1];
  return ans;
}

double latch1::Vcross(std::vector<double> &a, std::vector<double> &b){
  return a[0]*b[1]-a[1]*b[0];
}

double latch1::Vdot(std::vector<double> &a, std::vector<double> &b){
  return a[0]*b[0]+a[1]*b[1];
}

////////////////////////////////////////////////////////////////////////////////
// End Aux
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Conv
////////////////////////////////////////////////////////////////////////////////

std::vector<double> latch1::toCart(std::vector<double> &p, 
				   std::vector<double> &v){
  std::vector<double> xy (2,0.0);
  xy[0] = p[0]+(p[2]+v[1])*cos(v[0]);
  xy[1] = p[1]+(p[2]+v[1])*sin(v[0]);
  return xy;
}

void latch1::toModel(std::vector<double> &xy, std::vector<double> &p,
		     std::vector<double> &v){
  v[0] = atan2(xy[1]-p[1],xy[0]-p[0]);
  double dist = sqrt((xy[0]-p[0])*(xy[0]-p[0])+(xy[1]-p[1])*(xy[1]-p[1]));
  v[1] = dist-p[2]; // distance - r
}

////////////////////////////////////////////////////////////////////////////////
// End Conv
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Models
////////////////////////////////////////////////////////////////////////////////
// xp, yp, r, th_L, d_L
// th, d

//?
void latch1::slide(std::vector<double> &p, std::vector<double> &v,
		   std::vector<double> &action){
  // Calculate equilibrium point
  double th = 0;
  v[1] += action[0]*cos(th)+action[1]*sin(th);
}

void latch1::free(std::vector<double> &p, std::vector<double> &v,
		  std::vector<double> &action){
  // Calculate equilibrium point
  std::vector<double> start = toCart(p,v);
  start[0] += action[0];
  start[1] += action[1];
  toModel(start,p,v);
}

// with absolute action
void latch1::slideAbs(double &th, double &d, std::vector<double> &aAbs){
  d = aAbs[0]*cos(th)+aAbs[1]*sin(th);
}

void latch1::freeAbs(double &x, double &y, std::vector<double> &aAbs){
  x = aAbs[0];
  y = aAbs[1];
}
////////////////////////////////////////////////////////////////////////////////
// End Models
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Contain
////////////////////////////////////////////////////////////////////////////////

bool latch1::contains(std::vector<double> &P, std::vector<double> &A,
		      std::vector<double> &B, std::vector<double> &C){
  // Compute vectors
  std::vector<double> v0 = Vminus(C,A);
  std::vector<double> v1 = Vminus(B,A);
  std::vector<double> v2 = Vminus(P,A);

  // Compute dot products
  double dot00 = Vdot(v0,v0);
  double dot01 = Vdot(v0,v1);
  double dot02 = Vdot(v0,v2);
  double dot11 = Vdot(v1,v1);
  double dot12 = Vdot(v1,v2);

  // Compute barycentric coordinates
  double invDenom = 1/(dot00*dot11-dot01*dot01);
  double u = (dot11*dot02-dot01*dot12)*invDenom;
  double v = (dot00*dot12-dot01*dot02)*invDenom;

  // Check if point is in triangle
  return (u>=0) && (v>=0) && (u+v<=1);
}

////////////////////////////////////////////////////////////////////////////////
// End Contain
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Checks
////////////////////////////////////////////////////////////////////////////////

bool latch1::inLatch(std::vector<double> &p, std::vector<double> &v){

  // check if in latch
  double thTol = std::abs(atan2(_W/2,p[2]+p[4]-_L));
  double thDel = std::abs(v[0]-p[3]);
  if(thTol > thDel){
    // if you are exactly equal, you are outside the latch
    if(((v[1]+p[2])*cos(thDel) <= (p[2]+p[4])) && 
       ((v[1]+p[2])*cos(thDel) >= (p[2]+p[4]-_L))){
      // In latch
      return true;
    }
    else{
      // Not In Latch
      return false;
    }
  }
  else return false; // Not In Latch
}

////////////////////////////////////////////////////////////////////////////////
// End Checks
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Machine
////////////////////////////////////////////////////////////////////////////////
// xp, yp, r, th_L, d_L
// th, d

// s = 
// 0 - in latch
// 1 - free
// 2 - slide Left
// 3 - slide Right
// 4 - on circle


void latch1::machine(std::vector<double> &p, std::vector<double> &v,int &s,
		     std::vector<double> &aAbs){
  switch (s) {
  case 0:
    {
      // Don't need to set any variables beforehand
      v[0] = p[3];
      // make action relative to origin of slider
      std::vector<double> action = aAbs;
      action[0] -= p[0]+p[2]*cos(p[3]);
      action[1] -= p[1]+p[2]*sin(p[3]);
      
      // move handle
      slideAbs(p[3],v[1],action);
      if(v[1]>p[4]){
	v[1] = p[4];
      }
      else if(v[1]<(p[4]-_L)){
	v[1]=p[4]-_L;
	s = 1;
      }
      // no other else because variables are already set
      // the else would be returning s=0 again 
      // which is the terminating conditioning
    }
    break;
  case 1:
    {
      // Need to set variables beforehand

      // Define action triangle
      std::vector<double> pivot (2,0.0);
      pivot[0] = p[0];
      pivot[1] = p[1];
      std::vector<double> startPt = toCart(p,v);
      std::vector<double> endPt (2,0.0);
      freeAbs(endPt[0],endPt[1],aAbs); // COULD BE A PROBLEM
      //std::vector<double> end = toCart(p,v);

      // Define points in front of latch
      std::vector<double> pt0 (2,0.0); // left
      std::vector<double> pt1 (2,0.0); // right
      pt0[0] = p[0]+(p[2]+p[4])*cos(p[3])+(_W/2)*cos(p[3]+M_PI/2)
	+_L*cos(p[3]+M_PI);
      pt0[1] = p[1]+(p[2]+p[4])*sin(p[3])+(_W/2)*sin(p[3]+M_PI/2)
	+_L*sin(p[3]+M_PI);
      pt1[0] = p[0]+(p[2]+p[4])*cos(p[3])-(_W/2)*cos(p[3]+M_PI/2)
	+_L*cos(p[3]+M_PI);
      pt1[1] = p[1]+(p[2]+p[4])*sin(p[3])-(_W/2)*sin(p[3]+M_PI/2)
	+_L*sin(p[3]+M_PI);

      // Check if the action triangle contains the points
      bool contPt0 = contains(pt0,pivot,startPt,endPt);
      bool contPt1 = contains(pt1,pivot,startPt,endPt);

      // Check where the action started
      double thTol = std::abs(atan2(_W/2,p[2]+p[4]-_L));

      bool startL = v[0] >= (p[3]+thTol);
      bool startR = v[0] <= (p[3]-thTol);

      if(contPt0 && !contPt1){
	if(startL) s = 2;
	else s = 0;
      }
      else if(!contPt0 && contPt1){
	if(startR) s = 3;
	else s = 0;
      }
      else if(contPt0 && contPt1){
	if(startL) s = 2;
	else if(startR) s = 3;
      }
      else if(!contPt0 && !contPt1){      
	if(((endPt[0]-pivot[0])*(endPt[0]-pivot[0])
	    +(endPt[1]-pivot[1])*(endPt[1]-pivot[1]))
	   <(p[2]*p[2])) s = 4;
	else toModel(endPt,p,v); // stayed free. set p and v for after return
      }
    }
    break;
  case 2:
    {
      // Don't need to set any variables beforehand
    
      // set variables for transition
      v[0] = p[3]+std::abs(atan2(_W/2,p[2]+p[4]-_L)); // theta
      double dRMinSq = (p[2]+p[4]-_L)*(p[2]+p[4]-_L)+_W*_W/4;

      // make action relative to origin of slider
      std::vector<double> action = aAbs;
      action[0] -= p[0]+p[2]*cos(v[0]);
      action[1] -= p[1]+p[2]*sin(v[0]);

      // transition
      slideAbs(v[0],v[1],action);

      if(((v[1]+p[2])*(v[1]+p[2]))<dRMinSq){
	// slid past latch and should slip back to free
	v[1]=sqrt(dRMinSq)-p[2]-.000001; // the only square root. awful.
	s = 1;
      }
      // no other else because variables are already set
      // the else would be returning s=2 again 
      // which is the terminating conditioning
    } 
    break;
  case 3:
    {
      // Don't need to set any variables beforehand
      // set variables for transition
      v[0] = p[3]-std::abs(atan2(_W/2,p[2]+p[4]-_L)); // theta
      double dRMinSq = (p[2]+p[4]-_L)*(p[2]+p[4]-_L)+_W*_W/4;

      // make action relative to origin of slider
      std::vector<double> action = aAbs;
      action[0] -= p[0]+p[2]*cos(v[0]);
      action[1] -= p[1]+p[2]*sin(v[0]);

      // transition
      slideAbs(v[0],v[1],action);

      if(((v[1]+p[2])*(v[1]+p[2]))<dRMinSq){
	// slid past latch and should slip back to free
	v[1]=sqrt(dRMinSq)-p[2]-.000001; // the only square root. awful.
	s = 1;
      }
      // no other else because variables are already set
      // the else would be returning s=3 again 
      // which is the terminating conditioning 
    }
    break;
  case 4:
    {
      // Don't need to set any variables beforehand
      //double x = action[0]+state.params[2]*cos(state.vars[0]);
      //double y = action[1]+state.params[2]*sin(state.vars[0]);
      //nextState.vars[0] = atan2(y,x);
      if(true){
	// old way works pretty well
	v[0] = atan2(aAbs[1]-p[1],aAbs[0]-p[0]);
	v[1] = 0.0;
      }
      else{
	// hopefully new way works better
	// new way
	double thi = v[0];
	int numSteps = 360;
	double dth = 2*M_PI/numSteps;
	std::vector<double> E(numSteps);
	double r = p[2];
	double ax = aAbs[0]-(p[0]+(p[2]+v[1])*cos(v[0]));
	double ay = aAbs[1]-(p[1]+(p[2]+v[1])*sin(v[0]));
	double KxP = 100;
	double KyP = 400;
	
	for(size_t i=0;i<numSteps;i++){
	  double th = -M_PI+(i+1)*dth;
	  E[i]=0.5*KxP*pow((r*((cos(thi)-cos(th))*ay-(sin(thi)-sin(th))*ax)),2)
	    + 0.5*KyP*pow((r*((cos(thi)-cos(th))*ax+(sin(thi)-sin(th))*ay)
			   +ax*ax+ay*ay),2);
	}
	v[0] = -M_PI
	  +(std::distance(E.begin(),std::min_element(E.begin(),E.end()))+1)*dth;
	v[1] = 0.0;
      }
      // don't need to change state which is the terminating condition
      // This condition always ends the loop if reached
    }
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// End Machine
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Simulation
////////////////////////////////////////////////////////////////////////////////

void latch1::simulate(std::vector<double> &p, std::vector<double> &v,
		      std::vector<double> &action){
  // make action absolute
  std::vector<double> aAbs = action;
  std::vector<double> start = toCart(p,v);
  aAbs[0] += start[0];
  aAbs[1] += start[1];

  // Set start state of machine from state of mechanism
  int s;
  if(((aAbs[0]-p[0])*(aAbs[0]-p[0])
      +(aAbs[1]-p[1])*(aAbs[1]-p[1]))
     <(p[2]*p[2])) s = 4;
  else if(inLatch(p,v)) s = 0;
  else s = 1;

  // Simulate. The termination condition is that the machine state didn't change
  int sPrev = s;
  //std::cout << "start state: " << s << std::endl;
  while(true){
    //std::cout << "yo" << std::endl;
    machine(p,v,s,aAbs);
    /*
    std::cout << "current state: " << s << std::endl;
    std::cout << "p:" << p[0] << "," << p[1] << "," << p[2] << ","
	      << p[3] << "," << p[4] << std::endl;
    std::cout << "v:" << v[0] << "," << v[1] << std::endl;
    */
    if(s==sPrev) break;
    else sPrev=s;
  }
}

// Simulate the latch with action noise
void latch1::simulateWActionNoise(std::vector<double> &p, 
				  std::vector<double> &v,
				  std::vector<double> &action,
				  double &sig, double &mu){

  double x1 = ((double)rand()/(double)RAND_MAX);
  double x2 = ((double)rand()/(double)RAND_MAX);

  // Add noise to the action
  std::vector<double> aCopy = action;
  aCopy[0] += sqrt(-2*logUtils::safe_log(x1))*sin(2*M_PI*x2)*sig+mu;
  aCopy[1] += sqrt(-2*logUtils::safe_log(x1))*cos(2*M_PI*x2)*sig+mu;

  // Simulate with noisy action
  simulate(p,v,aCopy);
}

////////////////////////////////////////////////////////////////////////////////
// End Simulation
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Noise
////////////////////////////////////////////////////////////////////////////////

void latch1::addNoise(std::vector<double> &p, std::vector<double> &v, 
		      double &sig, double &mu){

  double x1 = ((double)rand()/(double)RAND_MAX);
  double x2 = ((double)rand()/(double)RAND_MAX);
  v[1] += sqrt(-2*logUtils::safe_log(x1))*sin(2*M_PI*x2)*sig+mu;
  if(v[1]<0) v[1]=0;
  v[0] += sqrt(-2*logUtils::safe_log(x1))*cos(2*M_PI*x2)
    *(sig/(p[2]+v[1]))
    +(mu/(p[2]+v[1])); // this last part is never used  

  // check if past latch
  double thTol = std::abs(atan2(_W/2,p[2]+p[4]-_L));
  double thDel = std::abs(v[0]-p[3]);
  if(thTol > thDel){
    // if you are exactly equal, you are outside the latch
    // Check if it's past
    if((v[1]+p[2])*cos(thDel) > (p[2]+p[4])) v[1]=(p[2]+p[4])/cos(thDel)-p[2];
  }
}

////////////////////////////////////////////////////////////////////////////////
// End Noise
////////////////////////////////////////////////////////////////////////////////

/*
int main(){
  std::vector<double> x1 (2,0.0);
  std::vector<double> x2 (2,2.0);
  std::vector<double> x3 (2,0.0);
  x3[0] = 1.0;
  std::vector<double> x4 (2,1.0);
  x4[1] = 5.0;
  std::cout << intLines(x1,x2,x3,x4) << std::endl;

  std::cout << "Dot product test" << std::endl;
  std::vector<double> x5 (2,1.0); // center
  std::vector<double> x6 (2,0.0); // start
  x6[0] = 1.99999;
  std::cout << Vdot(x5,x6) << std::endl;
  std::cout << Vdot(x5,x5) << std::endl;
  std::cout << "inside: " << (Vdot(x5,x6) < Vdot(x5,x5)) << std::endl;

  std::cout << "Slide absolute test" << std::endl;
  std::vector<double> a (2,1.0);
  a[0] = 2.0;
  std::vector<double> s (2,0.0);
  double th = M_PI/4;
  double d = 1.414;
  s[0]=d*cos(th);
  s[1]=d*sin(th);
  d += a[0]*cos(th)+a[1]*sin(th);
  std::cout << d << std::endl;
  std::vector<double> ABS (2,0.0);
  ABS[0] = s[0]+a[0];
  ABS[1] = s[1]+a[1];
  std::cout << ABS[0]*cos(th)+ABS[1]*sin(th) << std::endl;

  std::cout << "Contains test" << std::endl;
  std::vector<double> A (2,0.0);
  std::vector<double> B (2,0.0);
  B[0]=2.0;
  std::vector<double> C (2,0.0);
  C[1]=2.0;
  std::vector<double> P (2,0.0);
  P[1]=2.0;
  std::cout << contains(P,A,B,C) << std::endl;

  std::cout << "Machine test" << std::endl;
  // 7.5, 11.75, 14.75 inches
  // 0.19, 0.30, 0.375 m
  // xp, yp, r, th_L, d_L
  // th, d

  std::vector<double> p;
  p.push_back(0.0);
  p.push_back(-0.30);
  p.push_back(0.19);
  p.push_back(1.5708);
  p.push_back(0.11);

  std::vector<double> v;
  // start inside latch
  v.push_back(1.5708);
  v.push_back(0.11);

  // start outside latch
  //v.push_back(1.3+.25);
  //v.push_back(0.11);

  std::vector<double> action;
  action.push_back(-0.06);
  action.push_back(0.0);

  for(size_t i=0;i<p.size();i++){
    std::cout << p[i] << ",";
  }
  std::cout << std::endl;

  for(size_t i=0;i<v.size();i++){
    std::cout << v[i] << ",";
  }
  std::cout << std::endl;


  std::cout << "inLatch: " << inLatch(p,v) << std::endl; 

  std::vector< std::vector<double> > fakeActions;

  std::vector<double> actions0;
  actions0.push_back(-0.0848528);
  actions0.push_back(0.0848528);
  fakeActions.push_back(actions0);
  std::vector<double> actions1;
  actions1.push_back(-2.20436e-17);
  actions1.push_back(-0.12);
  fakeActions.push_back(actions1);
  std::vector<double> actions2;
  actions2.push_back(0.0848528);
  actions2.push_back(0.0848528);
  fakeActions.push_back(actions2);
  std::vector<double> actions3;
  actions3.push_back(-2.20436e-17);
  actions3.push_back(-0.12);
  fakeActions.push_back(actions3);
  std::vector<double> actions4;
  actions4.push_back(7.34788e-18);
  actions4.push_back(0.12);
  fakeActions.push_back(actions4);
  std::vector<double> actions5;
  actions5.push_back(-2.20436e-17);
  actions5.push_back(-0.12);
  fakeActions.push_back(actions5);
  std::vector<double> actions6;
  actions6.push_back(-0.0848528);
  actions6.push_back(0.0848528);
  fakeActions.push_back(actions6);
  std::vector<double> actions7;
  actions7.push_back(-0.0848528);
  actions7.push_back(-0.0848528);
  fakeActions.push_back(actions7);
  std::vector<double> actions8;
  actions8.push_back(0.0848528);
  actions8.push_back(0.0848528);
  fakeActions.push_back(actions8);
  std::vector<double> actions9;
  actions9.push_back(0.0848528);
  actions9.push_back(0.0848528);
  fakeActions.push_back(actions9);

  std::vector< std::vector<double> > fakeObs;

  std::vector<double> obs0;
  obs0.push_back(-0.000953149);
  obs0.push_back(-0.000111598);
  fakeObs.push_back(obs0);
  std::vector<double> obs1;
  obs1.push_back(8.88733e-05);
  obs1.push_back(-0.109452);
  fakeObs.push_back(obs1);
  std::vector<double> obs2;
  obs2.push_back(0.0769531);
  obs2.push_back(-0.0280716);
  fakeObs.push_back(obs2);
  std::vector<double> obs3;
  obs3.push_back(0.0738308);
  obs3.push_back(-0.127177);
  fakeObs.push_back(obs3);
  std::vector<double> obs4;
  obs4.push_back(0.0826914);
  obs4.push_back(-0.0121552);
  fakeObs.push_back(obs4);
  std::vector<double> obs5;
  obs5.push_back(0.0799746);
  obs5.push_back(-0.122961);
  fakeObs.push_back(obs5);
  std::vector<double> obs6;
  obs6.push_back(0.0416737);
  obs6.push_back(-0.0298375);
  fakeObs.push_back(obs6);
  std::vector<double> obs7;
  obs7.push_back(-0.0222209);
  obs7.push_back(-0.115916);
  fakeObs.push_back(obs7);
  std::vector<double> obs8;
  obs8.push_back(0.0582718);
  obs8.push_back(-0.0343393);
  fakeObs.push_back(obs8);
  std::vector<double> obs9;
  obs9.push_back(0.126293);
  obs9.push_back(0.0414726);
  fakeObs.push_back(obs9);



  std::cout << "As=[]" << std::endl;
  std::cout << "Os=[]" << std::endl;
  std::cout << "ps=[]" << std::endl;
  std::cout << "vs=[]" << std::endl;

  std::cout << "As.append([" << 0.0 << "," << 0.0 << "])" << std::endl;
  std::cout << "Os.append([" << 0.0 << "," << 0.0 << "])" << std::endl;
  std::cout << "ps.append([" << p[0] << "," << p[1] << "," << p[2] << ","
	      << p[3] << "," << p[4] << "])" << std::endl;
  std::cout << "vs.append([" << v[0] << "," << v[1] << "])" << std::endl;

  for(size_t i=0;i<10;i++){
    simulate(p,v,fakeActions[i]);
    std::cout << "As.append([" << fakeActions[i][0] << "," 
	      << fakeActions[i][1] << "])" << std::endl;
    std::cout << "Os.append([" << fakeObs[i][0] << "," 
	      << fakeObs[i][1] << "])" << std::endl;
    std::cout << "ps.append([" << p[0] << "," << p[1] << "," << p[2] << ","
	      << p[3] << "," << p[4] << "])" << std::endl;
    std::cout << "vs.append([" << v[0] << "," << v[1] << "])" << std::endl;
  }
  std::cout << "inLatch: " << inLatch(p,v) << std::endl; 

  std::cout << action[0] << "," << action[1] << std::endl;

  for(size_t i=0;i<p.size();i++){
    std::cout << p[i] << ",";
  }
  std::cout << std::endl;

  for(size_t i=0;i<v.size();i++){
    std::cout << v[i] << ",";
  }
  std::cout << std::endl;

  return 1;
}
*/
