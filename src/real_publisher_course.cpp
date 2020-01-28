#include <iostream>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "course_publisher");
	
	ros::NodeHandle nh;
	
	//the second parameter to advertise() is the size of the message queue used for publishing messages.	
	ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("course", 10);

	ros::Rate loop_rate(5);

	ackermann_msgs::AckermannDriveStamped ackermann_data;
	ackermann_data.drive.steering_angle = 5;
	ackermann_data.drive.speed = 0;

	int count = 2;

	while(ros::ok()) {
		ackermann_data.drive.steering_angle = count;
		ackermann_data.drive.speed = count;

		ackermann_pub.publish(ackermann_data);
		ROS_INFO("-------------------");
		ROS_INFO("STEERING ANGLE: %fdegree", ackermann_data.drive.steering_angle);
		ROS_INFO("SPEED: %fm/s", ackermann_data.drive.speed);
		
		//if there is no while
		//ros::spin(); 
		ros::spinOnce();

		loop_rate.sleep();
		count+=count;
	}
	
	return 0;
}
