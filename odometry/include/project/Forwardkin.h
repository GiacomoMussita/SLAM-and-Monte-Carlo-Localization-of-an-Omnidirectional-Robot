// Forwardkin.h

#ifndef FORWARDKIN_H
#define FORWARDKIN_H

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/CustomRpm.h"


class Forwardkin {
  public:
    Forwardkin();
    void computeControl(const geometry_msgs::TwistStamped& velmsg);
    void main_loop();

  private:
    ros::NodeHandle n; 
    ros::Subscriber sub;
    ros::Publisher pub;
    project::CustomRpm wheelsmsg; // message for the velocities of the wheels
    // parameters
    double radius;
    double length;
    double width;
    int gear_ratio;
    double pi;
    ros::Time previoustime; // simulation time at previous step (in seconds)
    bool isFirst; // variable to start compute command velocities from second time instant because at t=0 we have no velocities vx, vy, w
};

#endif