/* TODO: Also publish the state scaled to orbit. */
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense> 

#include "pv_estimator/State.h"
#include "pv_estimator/Meas.h"
#include "estimator.h"

#include <string>
#include <cstring>
#include <array>

Eigen::VectorXd z(4), state(4);//z is for measurement vector
Eigen::MatrixXd P(4,4);
Estimator estimator;

// bool editing,operating;
// ros::Rate waitRate(10000);

ros::Publisher pubState;
ros::Subscriber subMeas;

ros::Time tsMeas;

void measCallback(const pv_estimator::Meas msg){
  tsMeas = ros::Time(msg.tStamp);
  z(0) = msg.r[0];
  z(1) = msg.r[1];
  z(2) = msg.v[0];
  z(3) = msg.v[1];
}   


int main(int argc, char** argv){
  ros::init(argc,argv,"estimator");
  std::string robotName;
  ros::NodeHandle nh;
  tsMeas = ros::Time(0);

  subMeas = nh.subscribe(std::string("/meas"),1000,measCallback);
  pubState = nh.advertise<pv_estimator::State>(std::string("/state"),1000);
  ros::Rate loop_rate(100);
  ros::Time tsMeasOld;
  ROS_INFO("Estimator Node Initialized");

  while(ros::ok()){
    if(!estimator.initialized()){
      //Do things
      if(tsMeas.toSec() != 0){
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }
      estimator.initialize(z);
      ROS_INFO("Estimator Initialized");
    }else{// Predict and Correct
      double dtMeas = ((ros::Duration)(tsMeas - tsMeasOld)).toSec();
      estimator.predict(dtMeas);
      estimator.correct(z);
    }
    tsMeasOld = ros::Time().fromNSec(tsMeas.toNSec());
    
    // Fill message
    pv_estimator::State stateMsg;
    state = estimator.getState();
    for(int i=0;i<2;i++){
      stateMsg.r[i] = state(i);
      stateMsg.v[i] = state(i+2);
    }
    ros::Time tsNow = ros::Time::now();
    stateMsg.tStamp = tsNow.toSec();
    pubState.publish(stateMsg);

    // robot_controller::StateCov stateCovMsg;
    // P = estimator.getCovariance();
    // for(int i=0;i<P.rows();i++){
    //   stateCovMsg.P[i] = P(i,i);
    // }
    // stateCovMsg.tStamp = tsNow.toSec();
    // pubStateCov.publish(stateCovMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
