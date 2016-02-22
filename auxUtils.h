#ifndef AUX_UTILS_H
#define AUX_UTILS_H

#include <vector>

#include "stateStruct.h"

namespace auxUtils {

  bool nearlyColinear(std::vector<double> &a, std::vector<double> &b,
		      double &threshAngle);
  std::vector<double> Vminus(std::vector<double> &a, std::vector<double> &b);
  double Vcross(std::vector<double> &a, std::vector<double> &b);
  double Vdot(std::vector<double> &a, std::vector<double> &b);
  double VsqMag(std::vector<double> &a);
  void printState(stateStruct &state);
  void printAction(std::vector<double> &action);

}

#endif //AUX_UTILS_H
