// Kinematics class implementation

#include "project/Kinematics.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "rosgraph_msgs/Clock.h"
#include <dynamic_reconfigure/server.h>
#include "project/parametersConfig.h"
#include <cmath>

Kinematics::Kinematics() { // constructor definition
  
  this->sub = this->n.subscribe("wheel_states", 1000, &Kinematics::solveKinematics, this);
  this->pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  n.getParam("radius", radius);
  n.getParam("length", length);
  n.getParam("width", width);
  n.getParam("CPR", CPR);
  n.getParam("gear_ratio", gear_ratio);
  n.getParam("pi", pi);
  for(int i=0; i<4; ++i) { // initialize ticks to 0
    tick[i] = 0.0;
  }
  previoustime.sec = 0;
  previoustime.nsec = 0;
  isFirst = true;
}

void Kinematics::solveKinematics(const sensor_msgs::JointState& encordermsg) {

  // delta of the wheels ticks from the encoders (ticks/s)
  double dtick[4];
  // delta tempo (s)
  double dT;

  // when the bag restart, we need to reset the values of the encoder and the time
  // we realize a bag is restarted, when the previous time is bigger than the new time
  if(encordermsg.header.stamp.sec < previoustime.sec) {
    isFirst = true;
  }

  if (isFirst == true) {

    for(int i=0; i<4; ++i) {
      tick[i] = encordermsg.position[i];
    }
  }
  
  if(isFirst == false) {

    dT = (encordermsg.header.stamp.sec - previoustime.sec) + 
        (encordermsg.header.stamp.nsec - previoustime.nsec)*pow(10, -9);
    
    for (int i=0; i<4; ++i) {
      dtick[i] = encordermsg.position[i] - tick[i];            
      omega[i] = (dtick[i]*2*pi)/(dT*CPR*gear_ratio);
    }
    velmsg.twist.linear.x = radius*(omega[0] + omega[1] + omega[2] + omega[3])/4;
    velmsg.twist.linear.y = radius*(-omega[0] + omega[1] + omega[2] - omega[3])/4;
    velmsg.twist.angular.z = radius*(-omega[0] + omega[1] - omega[2] + omega[3])/(4*(length + width));
    for (int i=0; i<4; ++i) {
      tick[i] = encordermsg.position[i];
    }
  }

  isFirst = false;
  previoustime.sec = encordermsg.header.stamp.sec;
  previoustime.nsec = encordermsg.header.stamp.nsec;
  // publish also the time in the header so that all times in different nodes are synchronized
  velmsg.header.stamp.sec = encordermsg.header.stamp.sec;
  velmsg.header.stamp.nsec = encordermsg.header.stamp.nsec;
  this->pub.publish(velmsg);
}

void Kinematics::callbackServer(double* radius, int* CPR, double* length, double* width,
                    project::parametersConfig &config) {
  
  ROS_INFO("Radius updated from %f to %f", *radius, config.radius);
  ROS_INFO("Counts per revolution updated from %d to %d", *CPR, config.CPR);
  ROS_INFO("Length updated from %f to %f", *length, config.length);
  ROS_INFO("Width updated from %f to %f", *width, config.width);
  *radius = config.radius;
  *CPR = config.CPR;
  *length = config.length;
  *width = config.width;
}

void Kinematics::main_loop() {

  // dynamic server definition
  dynamic_reconfigure::Server<project::parametersConfig> dynServer; 
  dynamic_reconfigure::Server<project::parametersConfig>::CallbackType func;
  func = boost::bind(&Kinematics::callbackServer, this, &radius, &CPR, &length, &width,_1); 
  // define the callback of dynamic reconfigure server
  dynServer.setCallback(func);

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    ros::spinOnce();

    loop_rate.sleep();
  }
}