// Kinematics.h

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/parametersConfig.h"

class Kinematics {
  public:
    Kinematics();
    void solveKinematics(const sensor_msgs::JointState& encordermsg);
    void callbackServer(double* radius, int* CPR, double* length, double* width,
                        project::parametersConfig &config);
    void main_loop();

  private:
    ros::NodeHandle n; 
    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::TwistStamped velmsg; // message for the velocities (v,w) of the robot
    // parameters
    double radius;
    double length;
    double width;
    int CPR;
    int gear_ratio;
    double pi;
    double tick[4]; // encorders positions (in ticks)
    double omega[4]; // wheels speed (rad/s)
    ros::Time previoustime; // simulation time at previous step (in seconds)
    bool isFirst; // variable to start compute velocities from second time instant (no delta time and delta positions at t=0)
};

#endif