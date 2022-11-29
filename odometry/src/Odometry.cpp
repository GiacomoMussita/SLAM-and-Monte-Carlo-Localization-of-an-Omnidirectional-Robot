// Odometry class implementation

#include "project/Odometry.h"
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include "project/integModeConfig.h"
#include "project/reset.h"

Odometry::Odometry() { // constructor definition

	this->sub = n.subscribe("cmd_vel", 1000, &Odometry::integrate, this);
	this->pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	mode = 0;
	n.getParam("x0", xk);
	n.getParam("y0", yk);
	n.getParam("theta0", thetak);
	previoustime.sec = 0;
		previoustime.nsec = 0;
	isFirst = true;
}

void Odometry::integrate(const geometry_msgs::TwistStamped& velmsg) {
	
	// delta tempo (s)
	double dT;

	// when the bag restart, we need to reset the values of the encoder and the time
		// we realize a bag is restarted, when the previous time is bigger than the new time
		if(velmsg.header.stamp.sec < previoustime.sec) {

		isFirst = true;
		// reset the initial position from parameters server
		n.getParam("x0", xk);
		n.getParam("y0", yk);
		n.getParam("theta0", thetak);
		}

	if(isFirst == false) {

		dT = (velmsg.header.stamp.sec - previoustime.sec) + 
	    	(velmsg.header.stamp.nsec - previoustime.nsec)*pow(10, -9);
		if(mode == 0) { // if the integration method is Euler
			integrateEuler(velmsg, dT);
		}
		else { // if the integration method is RK
			integrateRK(velmsg, dT);
		}
	}
	isFirst = false;
	posemsg.pose.pose.position.x = xk;
	posemsg.pose.pose.position.y = yk;
	// quaternions pose definition
	posemsg.pose.pose.orientation.x = 0;
	posemsg.pose.pose.orientation.y = 0;
	posemsg.pose.pose.orientation.z = sin(thetak/2);
	posemsg.pose.pose.orientation.w = cos(thetak/2);
	previoustime.sec = velmsg.header.stamp.sec;
	previoustime.nsec = velmsg.header.stamp.nsec;
	posemsg.header.stamp.sec = velmsg.header.stamp.sec;
	posemsg.header.stamp.nsec = velmsg.header.stamp.nsec;
	posemsg.header.frame_id = "odom";
	posemsg.child_frame_id = "base_link";
	this->pub.publish(posemsg);
}

void Odometry::integrateEuler(const geometry_msgs::TwistStamped& velmsg, double dT) {

	xk = xk + dT*(velmsg.twist.linear.x*cos(thetak) - velmsg.twist.linear.y*sin(thetak));
	yk = yk + dT*(velmsg.twist.linear.x*sin(thetak) + velmsg.twist.linear.y*cos(thetak));
	thetak = thetak + dT*velmsg.twist.angular.z;
}

void Odometry::integrateRK(const geometry_msgs::TwistStamped& velmsg, double dT) {

	xk = xk + dT*(velmsg.twist.linear.x*cos(thetak + velmsg.twist.angular.z*dT*0.5) 
		- velmsg.twist.linear.y*sin(thetak + velmsg.twist.angular.z*dT*0.5));
	yk = yk + dT*(velmsg.twist.linear.x*sin(thetak + velmsg.twist.angular.z*dT*0.5)
		+ velmsg.twist.linear.y*cos(thetak + velmsg.twist.angular.z*dT*0.5));
	thetak = thetak + dT*velmsg.twist.angular.z;
}

bool Odometry::resetOdometry(double* xk, double* yk, double* thetak, project::reset::Request &req,
					project::reset::Response &res) {

	res.old_x = *xk;
	res.old_y = *yk;
	res.old_theta = *thetak;
	*xk = req.new_x;
	*yk = req.new_y;
	*thetak = req.new_theta;
	ROS_INFO("Modified pose to: x = %f, y = %f, theta = %f", *xk, *yk, *thetak);
	return true;
}

void Odometry::callbackServer(int* mode, project::integModeConfig &config) {
	
	ROS_INFO("Integration method changed from %d to %d", *mode, config.integration_mode);
	*mode = config.integration_mode;
}

void Odometry::main_loop() {

	// server
	ros::ServiceServer poseService = n.advertiseService<project::reset::Request, 
		project::reset::Response>
		("reset", boost::bind(&Odometry::resetOdometry, this, &xk, &yk, &thetak, _1, _2));

	// dynamic server definition
	dynamic_reconfigure::Server<project::integModeConfig> dynServer; 
	dynamic_reconfigure::Server<project::integModeConfig>::CallbackType func;
	func = boost::bind(&Odometry::callbackServer, this, &mode, _1); // define the callback of dynamic reconfigure server
	dynServer.setCallback(func);

	ros::Rate loop_rate(10);

		while (ros::ok()) {

  		ros::spinOnce();

  		loop_rate.sleep();
	}
}
