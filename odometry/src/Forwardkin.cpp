// Forwardkin class implementation

#include "project/Forwardkin.h"
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/CustomRpm.h"

Forwardkin::Forwardkin() { // constructor definition

  this->sub = n.subscribe("cmd_vel", 1000, &Forwardkin::computeControl, this);
  this->pub = n.advertise<project::CustomRpm>("wheels_rpm", 1000);
  n.getParam("radius", radius);
  n.getParam("length", length);
  n.getParam("width", width);
  n.getParam("gear_ratio", gear_ratio);
  n.getParam("pi", pi);
  previoustime.sec = 0;
  previoustime.nsec = 0;
  isFirst = true;
}

void Forwardkin::computeControl(const geometry_msgs::TwistStamped& velmsg) {

  // when the bag restart, we need to reset the values of the encoder and the time
  // we realize a bag is restarted, when the previous time is bigger than the new time
  if(velmsg.header.stamp.sec < previoustime.sec) {
    isFirst = true;
  }

  if(isFirst == false) {

    wheelsmsg.header.stamp.sec = velmsg.header.stamp.sec;
    wheelsmsg.header.stamp.nsec = velmsg.header.stamp.nsec;
    // compute wheels speed in RPM
    wheelsmsg.rpm_fl = 60*gear_ratio*(((-length - width)*velmsg.twist.angular.z) 
      + velmsg.twist.linear.x - velmsg.twist.linear.y)/(radius);
    wheelsmsg.rpm_fr = 60*gear_ratio*(((length + width)*velmsg.twist.angular.z) 
      + velmsg.twist.linear.x + velmsg.twist.linear.y)/(radius);
    wheelsmsg.rpm_rr = 60*gear_ratio*(((length + width)*velmsg.twist.angular.z) 
      + velmsg.twist.linear.x - velmsg.twist.linear.y)/(radius);
    wheelsmsg.rpm_rl = 60*gear_ratio*(((-length - width)*velmsg.twist.angular.z) 
      + velmsg.twist.linear.x + velmsg.twist.linear.y)/(radius);
    this->pub.publish(wheelsmsg);
  }
  isFirst = false;
  previoustime.sec = velmsg.header.stamp.sec;
  previoustime.nsec = velmsg.header.stamp.nsec;
}

void Forwardkin::main_loop() {

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }  
}
