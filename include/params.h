#ifndef PARAMS_H
#define PARAMS_H

#include <Eigen/Dense>
#include <math.h>

class Params{
public:
  Params();
  Eigen::MatrixXd R, Q;
};
#endif