#ifndef LOG_UTILS_H
#define LOG_UTILS_H

#include <vector>

namespace logUtils {
  
  double safe_log(double x);

  double safe_exp(double x);

  double logSumExp(std::vector<double> pLog);
  
  std::vector<double> normalizeVectorInLogSpace(std::vector<double> pLog);
  
  std::vector<double> expLogProbs(std::vector<double> logProbs);
  
  //Ver 3: pass in vectors and inverted covariance matrix and determinant of covariance matrix
  double evaluteLogMVG(std::vector<double>& sampleVecVect, std::vector<double>& meanVecVect, std::vector<double>& invCovMatVect, double detCovMat);

}
  
#endif //LOG_UTILS_H
