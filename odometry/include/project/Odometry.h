// Odometry.h

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project/integModeConfig.h"
#include "project/reset.h"

class Odometry {
	public:
		Odometry();
		void integrate(const geometry_msgs::TwistStamped& velmsg);
		void integrateEuler(const geometry_msgs::TwistStamped& velmsg, double dT);
		void integrateRK(const geometry_msgs::TwistStamped& velmsg, double dT);
		bool resetOdometry(double* xk, double* yk, double* thetak, project::reset::Request &req,
							project::reset::Response &res);
		void callbackServer(int* mode, project::integModeConfig &config);
		void main_loop();

	private:
		ros::NodeHandle n; 
		ros::Subscriber sub;
		ros::Publisher pub;
		nav_msgs::Odometry posemsg;
		double xk;
		double yk;
		double thetak;
		double dT;
		int mode; // integration method
  		ros::Time previoustime; // simulation time at previous step (in seconds)
  		bool isFirst; // variable to start compute odometry from second time instant (no delta time and delta positions at t=0)
};

#endif