#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <vector>
#include <string>

namespace fileUtils {

  std::vector<double> convStrToVect(std::string &line);

  void txtFileToActionsObs(std::string fileName,std::vector<std::vector<double> > &actions,std::vector<std::vector<double> > &obs,int numSteps);

}

#endif //FILE_UTILS_H
