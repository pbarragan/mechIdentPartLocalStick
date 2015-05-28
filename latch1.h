#ifndef LATCH1_H
#define LATCH1_H

#include <vector>

namespace latch1 {
  std::vector<double> Vminus(std::vector<double> &a, 
			     std::vector<double> &b);
  double Vcross(std::vector<double> &a, std::vector<double> &b);
  double Vdot(std::vector<double> &a, std::vector<double> &b);
  std::vector<double> toCart(std::vector<double> &p, std::vector<double> &v);
  void toModel(std::vector<double> &xy, std::vector<double> &p,
	       std::vector<double> &v);
  void slide(std::vector<double> &p, std::vector<double> &v,
	     std::vector<double> &action);
  void free(std::vector<double> &p, std::vector<double> &v,
	    std::vector<double> &action);
  void slideAbs(double &th, double &d, std::vector<double> &aAbs);
  void freeAbs(double &x, double &y, std::vector<double> &aAbs);
  bool contains(std::vector<double> &P, std::vector<double> &A,
		std::vector<double> &B, std::vector<double> &C);
  bool inLatch(std::vector<double> &p, std::vector<double> &v);
  void machine(std::vector<double> &p, std::vector<double> &v,int &s,
	       std::vector<double> &aAbs);
  void simulate(std::vector<double> &p, std::vector<double> &v,
		std::vector<double> &action);
  void simulateWActionNoise(std::vector<double> &p, 
			    std::vector<double> &v,
			    std::vector<double> &action,
			    double &sig, double &mu);
  void addNoise(std::vector<double> &p, std::vector<double> &v, 
		double &sig, double &mu);
}
  
#endif //LATCH1_H
