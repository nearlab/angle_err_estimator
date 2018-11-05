#include "simulator.h"

Eigen::VectorXd measSimulator(const Eigen::VectorXd& state, const Params& params){
  Eigen::VectorXd z(4);
  z << state;
  return z;
}

