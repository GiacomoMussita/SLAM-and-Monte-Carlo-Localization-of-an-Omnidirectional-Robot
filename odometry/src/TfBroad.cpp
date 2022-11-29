// TfBroad class implementation

#include "project/TfBroad.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

TfBroad::TfBroad() { // constructor definition

  this->sub = n.subscribe("odom", 1000, &TfBroad::callback_frame, this);
}

void TfBroad::callback_frame(const nav_msgs::Odometry::ConstPtr& posemsg) {

  // set header
  transformStamped.header.stamp = posemsg -> header.stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  // set x,y
  transformStamped.transform.translation.x = posemsg -> pose.pose.position.x;
  transformStamped.transform.translation.y = posemsg -> pose.pose.position.y;
  transformStamped.transform.translation.z = 0.0;
  // set theta
  /*
  tf2::Quaternion q;
  q.setRPY(0, 0, posemsg-> );
  */
  transformStamped.transform.rotation.x = posemsg -> pose.pose.orientation.x;
  transformStamped.transform.rotation.y = posemsg -> pose.pose.orientation.y;
  transformStamped.transform.rotation.z = posemsg -> pose.pose.orientation.z;
  transformStamped.transform.rotation.w = posemsg -> pose.pose.orientation.w;
  // send transform
  br.sendTransform(transformStamped);
}
