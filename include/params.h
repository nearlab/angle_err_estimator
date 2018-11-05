#ifndef PARAMS_H
#define PARAMS_H

#include <Eigen/Dense>
#include <math.h>

class Params{
public:
  Params();
  Eigen::MatrixXd R, Q;
  double sigmaR, sigmaQ;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
};
#endif