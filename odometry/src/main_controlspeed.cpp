#include "ros/ros.h"
#include "project/Forwardkin.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "controlspeed");

  Forwardkin forwardKin;

  forwardKin.main_loop();

  return 0;
}