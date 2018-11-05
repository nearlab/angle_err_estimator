#include "simulator.h"

Eigen::VectorXd measSimulator(const Eigen::VectorXd& state, Params& params, const bool& noise){
  Eigen::VectorXd z(4);
  z << state;
  if(noise){
    Eigen::VectorXd zn(4);
    for(int i=0;i<4;i++){
      zn(i) = params.distribution(params.generator);
    }
    z+=zn;
  }
  return z;
}

