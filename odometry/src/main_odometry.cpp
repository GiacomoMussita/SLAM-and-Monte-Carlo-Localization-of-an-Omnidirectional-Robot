#include "ros/ros.h"
#include "project/Odometry.h"

int main(int argc, char **argv) {

  	ros::init(argc, argv, "odometry");

  	Odometry odo;

  	odo.main_loop();

	return 0;
}