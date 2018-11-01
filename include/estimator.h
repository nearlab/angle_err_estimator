#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <boost/array.hpp>
#include <math.h>
#include "params.h"
#include "simulator.h"
#include <ros/ros.h>

class Estimator{
public:
  Estimator();
  void predict(const double& dt);
  void correct(const Eigen::VectorXd& z);
  Eigen::MatrixXd parseMeas(const Eigen::VectorXd& z);
  void initialize(const Eigen::VectorXd& z);
  Eigen::VectorXd getState();
  Eigen::MatrixXd getCovariance();
  Params getParams();
  bool initialized();

private:
  Eigen::VectorXd state;  
  Eigen::MatrixXd P;
  bool isInitialized;
  Params params;
};
#endif
