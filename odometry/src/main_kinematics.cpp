#include "ros/ros.h"
#include "project/Kinematics.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "kinematics");

  Kinematics kin;

  kin.main_loop();

  return 0;
}