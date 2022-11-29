#include "ros/ros.h"
#include "project/TfBroad.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "broadcaster");

  TfBroad my_tf_broadcaster;

  ros::spin();

  return 0;
}
