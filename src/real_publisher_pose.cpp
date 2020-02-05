#include <iostream>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_publisher");
	
	ros::NodeHandle nh;
	
	//the second parameter to advertise() is the size of the message queue used for publishing messages.	
	ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 20);

	ros::Rate loop_rate(5);



	geometry_msgs::PoseStamped pose_data;
	pose_data.pose.position.x = 302457.243910768;
	pose_data.pose.position.y = 4123705.91286103;

	int count = 1;

	while(ros::ok()) {
		
		pose_data.pose.position.x += 0.1;
		pose_data.pose.position.y += 0.1;
		
		pose_pub_.publish(pose_data);
		ROS_INFO("-------------------");
		ROS_INFO("POSE_X: %f", pose_data.pose.position.x);
		ROS_INFO("POSE_Y: %f", pose_data.pose.position.y);
		
		count++;

		//if there is no while
		//ros::spin(); 
		ros::spinOnce();

		loop_rate.sleep();
	}
	
	return 0;
}
