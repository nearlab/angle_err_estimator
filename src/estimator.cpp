#include "estimator.h"

Estimator::Estimator(){
  this->params = Params();
  this->isInitialized = false;
  this->state = Eigen::VectorXd::Zero(4);
  this->P = Eigen::MatrixXd::Identity(4,4);
}
void Estimator::predict(const double& dt){
  Eigen::VectorXd r(2),v(2),rk(2),vk(2);
  r = state.segment(0,2);
  v = state.segment(2,2);
  rk = r + v*dt;
  vk = v;

  state.segment(0,2) = rk;
  state.segment(2,2) = vk;

  //Update error covariance
  Eigen::MatrixXd F,M;
  F = Eigen::MatrixXd(4,4);
  M = Eigen::MatrixXd(4,4);
  Eigen::Matrix3d z2 = Eigen::MatrixXd::Zero(2,2);
  Eigen::Matrix3d i2 = Eigen::MatrixXd::Identity(2,2);
  F << i2, i2*dt,
       z2, i2;
  M << 1/2*dt*dt*i2, z2
       z2          , dt*i2;

  this->P = F*this->P*F.transpose() + M*this->params.Q*M.transpose();
}
void Estimator::correct(const Eigen::VectorXd& z){
  Eigen::MatrixXd H = parseMeas(z);
  Eigen::MatrixXd R = this->params.R;
  Eigen::MatrixXd Y = H*this->P*H.transpose() + R;
  Eigen::MatrixXd C = this->P*H.transpose();
  Eigen::MatrixXd K = C*Y.inverse();

  Eigen::VectorXd zHatError = (z - measSimulator(this->state,this->params));
  Eigen::VectorXd dx = K*zHatError;

  this->state += dx;
  this->P = (Eigen::MatrixXd::Identity(12,12)-K*H)*this->P*(Eigen::MatrixXd::Identity(12,12)-K*H).transpose()+K*R*K.transpose();
  this->P = .5*(this->P + (this->P).transpose());
}
Eigen::MatrixXd Estimator::parseMeas(const Eigen::VectorXd& zMarkersRaw){
  //Returns the Jacobian for the measurement equation
  Eigen::MatrixXd H(4,4);
  H.setIdentity();
  return H;
}
void Estimator::initialize(const Eigen::VectorXd& z){
  this.state.head(4) << z.head(4);
  this->isInitialized = true;
}
Eigen::VectorXd Estimator::getState(){
  return this->state;
}
Eigen::MatrixXd Estimator::getCovariance(){
  return this->P;
}
Params Estimator::getParams(){
  return this->params;
}
bool Estimator::initialized(){
  return this->isInitialized;
}
