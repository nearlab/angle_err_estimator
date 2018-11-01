/* TODO: MAKE THESE COME FROM A LAUNCH OR CONFIG FILE. MAYBE READ IN DATA FROM VICON FOR MARKERLOCS */
#include "params.h"

Params::Params(){
  R = Eigen::MatrixXd(4,4);
  Q = Eigen::MatrixXd(4,4);
  double pi = 3.14159265358979;

  //Set up dynamics covariance matrix
  this->Q.setIdentity();

  //Set up measurement covariance matrix
  this->R.setIdentity();
