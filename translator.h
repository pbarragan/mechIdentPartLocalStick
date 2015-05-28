#ifndef TRANSLATOR_H
#define TRANSLATOR_H

#include <vector>
#include "stateStruct.h"

namespace translator {

  stateStruct stateTransition(stateStruct& state, std::vector<double>& action);

  std::vector<double> translateStToObs(stateStruct& state);

  bool isStateValid(stateStruct& state,
		    std::vector< std::vector<double> >& workspace);
}
  
#endif //TRANSLATOR_H
