#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "fileUtils.h"

std::vector<double> fileUtils::convStrToVect(std::string &line){
  std::string comma = ",";
  int firstCommaInd = line.find(comma);
  double x,y;
	
  std::istringstream ix(line.substr(0,firstCommaInd));
  ix >> x;
  
  std::istringstream iy(line.substr(firstCommaInd+1,line.length()-1));
  iy >> y;	
	
  std::vector<double> hold;
  hold.push_back(x);
  hold.push_back(y);
	
  std::cout << hold[0] << "," << hold[1] << "," << hold.size() << std::endl;

  return hold;
}

// this works for files as of 2/3/2015
void fileUtils::txtFileToActionsObs(std::string fileName,std::vector<std::vector<double> > &actions,std::vector<std::vector<double> > &obs,int numSteps) {
  // This function reads in a .txt file output from the realWorld.cpp
  // to populate the fake actions and observations for a new experiment

  // clear the actions and observations vectors
  actions.clear();
  obs.clear();
	
  // This is the setup to read the file containing the data
  std::string line;
  std::ifstream myfile(fileName.c_str());

  int start = 400096;//320087;
  int AOspace = 12;
  int stepSpace = 450099;//360083;
  int count = 1;

  //320087 -2.20436e-17,-0.12, action
  //320099 0.00061114,0.002612, obs
  //680170 0.12,0, action
  //680182 0.00133904,0.00395064, obs

  //This loop will read the file line by line, convert each number, and build a vector
  if (myfile.is_open())
    {
      while(myfile.good())
	{
	  getline(myfile,line);
	  
	  if(((count-start) % stepSpace) == 0 && actions.size() < numSteps){
	    std::cout << "action: " << count << std::endl;
	    actions.push_back(convStrToVect(line));
	  }
	  else if(((count-start-AOspace) % stepSpace) == 0 &&
		  obs.size() < numSteps){
	    std::cout << "obs: " << count << std::endl;
	    obs.push_back(convStrToVect(line)); 
	  }
	  count++;
	}
      std::cout << count << std::endl;
      myfile.close();
      std::cout << "txtFileToVector: Input file opened successfully" 
		<< std::endl;
      std::cout << "number of actions:" << actions.size() << std::endl;
      std::cout << "number of observations:" << obs.size() << std::endl;
    }
  
  else{
    std::cout << "txtFileToVector: Unable to open input file" << std::endl;
  }
}

/*
int main(){
  std::vector<std::vector<double> > fakeActions;
  std::vector<std::vector<double> > fakeObs;
  std::string fileName = "/home/barragan/data12112014new//data/2015_02_03/data1Tue_Feb__3_12_23_46_2015.txt";
  txtFileToActionsObs(fileName,fakeActions,fakeObs,10);

  return 1;
}
*/
