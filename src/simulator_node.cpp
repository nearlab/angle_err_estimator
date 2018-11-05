#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense> 

#include "pv_estimator/State.h"
#include "pv_estimator/Meas.h"
#include "estimator.h"

#include <math.h>

ros::Publisher pubState, pubMeas;

int main(int argc, char** argv){
  ros::init(argc,argv,"simulator");
  std::string robotName;
  ros::NodeHandle nh;
  tsMeas = ros::Time(0);

  pubMeas = nh.advertise<pv_estimator::Meas>(std::string("/meas"),1000,measCallback);
  pubState = nh.advertise<pv_estimator::State>(std::string("/state_truth"),1000);
  ros::Rate loop_rate(100);
  ROS_INFO("Simulation Node Initialized");

  ros::Time t(0);
  double T = 5;//Period of circle
  double r = 2;
  double pi = 3.1415926536;
  Params params();

  while(ros::ok()){
    Eigen::VectorXd z(4), state(4);

    double rx = r*cos(t*2*pi/T);
    double ry = r*sin(t*2*pi/T);
    double vx = -r*2*pi/T*sin(t*2*pi/T);
    double vy = r*2*pi*T*cos(t*2*pi/T);
    
    state << rx,ry,vx,vy;
    z << measSimulator(state,params);
    // Fill messages
    pv_estimator::State stateMsg;
    pv_estimator::Meas measMsg;
    for(int i=0;i<2;i++){
      stateMsg.r[i] = state(i);
      stateMsg.v[i] = state(i+2);
      measMsg.r[i] = z(i);
      measMsg.v[i] = z(i+2);
    }
    ros::Time tsNow = ros::Time::now();
    stateMsg.tStamp = tsNow.toSec();
    measMsg.tStamp = tsNow.toSec();
    pubState.publish(stateMsg);
    pubMeas.public(measMsg);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
}