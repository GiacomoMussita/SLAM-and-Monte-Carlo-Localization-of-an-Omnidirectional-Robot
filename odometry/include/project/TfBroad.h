// TfBroad.h

#ifndef TFBROAD_H
#define TFBROAD_H

#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

class TfBroad {
public:
  TfBroad();
  void callback_frame(const nav_msgs::Odometry::ConstPtr& posemsg);

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};

#endif