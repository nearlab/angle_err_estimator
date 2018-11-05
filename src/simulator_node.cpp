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

  pubMeas = nh.advertise<pv_estimator::Meas>(std::string("/meas"),1000);
  pubState = nh.advertise<pv_estimator::State>(std::string("/state_truth"),1000);
  ros::Rate loop_rate(100);
  ROS_INFO("Simulation Node Initialized");

  double tStart = ros::Time::now().toSec();
  double T = 5;//Period of circle
  double r = 2;
  double pi = 3.1415926536;
  Params params = Params();

  while(ros::ok()){
    Eigen::VectorXd z(4), state(4);

    double t = ros::Time::now().toSec() - tStart;
    state(0) = r*cos(t*2*pi/T);
    state(1) = r*sin(t*2*pi/T);
    state(2) = -r*2*pi/T*sin(t*2*pi/T);
    state(3) = r*2*pi*T*cos(t*2*pi/T);
    
    z << measSimulator(state,params,true);
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
    pubMeas.publish(measMsg);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
}