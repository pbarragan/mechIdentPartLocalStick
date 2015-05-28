#ifndef MODEL_UTILS_H
#define MODEL_UTILS_H

#include <vector>
#include "stateStruct.h"
#include "bayesFilter.h"


namespace modelUtils {

  // Particle Filter
  std::vector<double> calcFilterBankProbs(std::vector<BayesFilter>& filterBank);
  std::vector<double> calcFilterBankProbsLog(std::vector<BayesFilter>& filterBank);
  void findFilterBankBestGuesses(std::vector<BayesFilter>& filterBank,std::vector<stateStruct>& bestStates,std::vector<double>& bestStatesProbs);
  void normalizeAcrossFilters(std::vector<BayesFilter>& filterBank);

  //Overloaded
  std::vector<double> calcModelParamProbLog(std::vector<stateStruct>& stateList,std::vector<double>& probList);
  std::vector<double> calcModelParamProbLog(std::vector<stateStruct>& stateList,std::vector<double>& probList,std::vector<stateStruct>& modelParamPairs);
  
  std::vector<double> calcModelParamProbLogWOExp(std::vector<stateStruct>& stateList,std::vector<double>& probList,std::vector<stateStruct>& modelParamPairs);

  std::vector<double> calcModelProb(std::vector<stateStruct>& stateList, std::vector<double>& probList);

}
#endif //MODEL_UTILS_H
