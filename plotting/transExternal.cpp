#include <iostream>
#include <stdlib.h>

#include "../translator.h"
#include "../globalVars.h"

int main(int argc, char* argv[])
{
  if (argc > 11 || argc < 6) { // We expect 3 arguments: the program name, the model number, the number of iterations, writeOutFile
    std::cerr << "Usage: " << argv[0] << "NUMBER OF ITERATIONS" << std::endl;
  }
  else {
    // 2nd and 3rd are the action vector
    std::vector<double> action;
    action.push_back(atof(argv[1]));
    action.push_back(atof(argv[2]));
    
    // 4th is the model. the rest are dependent
    stateStruct state;
    state.model = atoi(argv[3]);

    for(size_t i=4;i<(MODEL_DESCRIPTIONS[state.model][0]+4);i++){
      state.params.push_back(atof(argv[i]));
    }
    for(size_t i=(MODEL_DESCRIPTIONS[state.model][0]+4);
	i<(MODEL_DESCRIPTIONS[state.model][1]
	   +MODEL_DESCRIPTIONS[state.model][0]+4);i++){
      state.vars.push_back(atof(argv[i]));
    }

    stateStruct nextState = translator::stateTransition(state,action);
    std::vector<double> obs = translator::translateStToObs(nextState);

    std::cout << nextState.model << "\n";
    for(size_t i=0;i<nextState.params.size();i++){
      std::cout << nextState.params[i];
      if(i!=(nextState.params.size()-1)) std::cout << ",";
    }
    std::cout << "\n";
    
    for(size_t i=0;i<nextState.vars.size();i++){
      std::cout << nextState.vars[i];
      if(i!=(nextState.vars.size()-1)) std::cout << ",";  
    }
    std::cout << "\n";
    std::cout << obs[0] << "," << obs[1];
  }
  return 1;
}
