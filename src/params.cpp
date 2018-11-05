/* TODO: MAKE THESE COME FROM A LAUNCH OR CONFIG FILE. MAYBE READ IN DATA FROM VICON FOR MARKERLOCS */
#include "params.h"

Params::Params(){
  this->R = Eigen::MatrixXd(4,4);
  this->Q = Eigen::MatrixXd(4,4);
  this->sigmaR = 1e-2;
  this->sigmaQ = 1e-3;
  this->distribution = std::normal_distribution<double>(0,sigmaR);

  //Set up dynamics covariance matrix
  this->Q.setIdentity()*sigmaQ*sigmaQ;

  //Set up measurement covariance matrix
  this->R.setIdentity()*sigmaR*sigmaR;
}