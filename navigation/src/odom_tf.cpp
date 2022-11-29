#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class TfBroad
{

  private:

  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher pub; 
 	
  public:

  TfBroad(){

    	this->sub = n.subscribe("/odom", 1, &TfBroad::callback, this);
  
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 
      msg->pose.pose.position.z));
    transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
  
  }

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "odom_tf");
 	TfBroad my_tf_broadcaster;
 	ros::spin();
 	return 0;
}