
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>
#include <fstream>
#include <string>
#define _USE_MATH_DEFINES

class TrajectoryVisualizer {
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher waypoint_pub_;
	ros::Publisher trajectory_pub_;
	ros::Publisher curcourse_viz_pub_;
	ros::Publisher ctrl_viz_pub_;
	ros::Subscriber pose_sub_;
	ros::Subscriber course_sub_;
	ros::Subscriber ctrl_cmd_steer_sub_;
	
	std::ifstream is_;

	std::string file_path_;
	std::vector<geometry_msgs::Point> waypoints_;
	std::vector<geometry_msgs::Point> vehicle_pose_;
	
	int pose_x_offset_, pose_y_offset_;
	int waypoint_x_offset_, waypoint_y_offset_;

	float cur_course_;
	float ctrl_cmd_steer_;

	geometry_msgs::Point temp_pose_;
	geometry_msgs::Point temp_pose_test_;

	int count = 0;
		

public:
	TrajectoryVisualizer(void):private_nh_("~") {
		initSetup();	
	}
	void initSetup(void) {
		waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/waypoint", 1);
		trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory", 1);
		curcourse_viz_pub_ = nh_.advertise<nav_msgs::Odometry>("/curcourseViz", 1);
		ctrl_viz_pub_ = nh_.advertise<nav_msgs::Odometry>("/ctrlsteerViz", 1);
		pose_sub_ = nh_.subscribe("/current_pose", 10, &TrajectoryVisualizer::poseCallback, this);
		course_sub_ = nh_.subscribe("/course", 10, &TrajectoryVisualizer::courseCallback, this);
		ctrl_cmd_steer_sub_ = nh_.subscribe("/ctrl_cmd", 10, &TrajectoryVisualizer::ctrl_cmd_steerCallback, this);
		initParam();
		getWaypoints();
		run();
	}
	void initParam(void) {
		private_nh_.getParam("file_path", file_path_);
		private_nh_.getParam("pose_x_offset", pose_x_offset_);
		private_nh_.getParam("pose_y_offset", pose_y_offset_);
		private_nh_.getParam("waypoint_x_offset", waypoint_x_offset_);
		private_nh_.getParam("waypoint_y_offset", waypoint_y_offset_);
	}
	void poseCallback(const geometry_msgs::PoseStampedConstPtr &odom_msg) {
		visualization_msgs::Marker trajectory;
		
		if (count != 0) {
			vehicle_pose_.push_back(temp_pose_test_);
		}		

		temp_pose_.x = odom_msg->pose.position.x - pose_x_offset_;
		temp_pose_.y = odom_msg->pose.position.y - pose_y_offset_;
		
		vehicle_pose_.push_back(temp_pose_);		

		temp_pose_test_.x = temp_pose_.x;
		temp_pose_test_.y = temp_pose_.y;
			
		trajectory.header.frame_id = "map";
		trajectory.header.stamp.fromSec(ros::Time::now().toSec());
		trajectory.ns = "trajectory";
		trajectory.id = 0;
		trajectory.action = visualization_msgs::Marker::ADD;
		trajectory.pose.orientation.w = 1.0;
		// visualization_msgs::Marker::LINE_STRIP
		// visualization_msgs::Marker::SPHERE_LIST
		trajectory.type = visualization_msgs::Marker::ARROW;
		trajectory.scale.x = 0.2;
		trajectory.scale.y = 0.3;
		trajectory.scale.z = 0.2;
		trajectory.color.g = 1.0f;
		trajectory.color.a = 1.0f;

		trajectory.points = vehicle_pose_;
		
		vehicle_pose_.clear();
				
		std::cout<<"CURRENT TRAJECTORY CONSISTS OF "<<trajectory.points.size()<<"POSES."<<std::endl;

		trajectory_pub_.publish(trajectory);

		count++;
	}

	void courseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
		cur_course_ = course_msg->drive.steering_angle;
		cur_course_ = (cur_course_*M_PI)/180;
     
	}

	void ctrl_cmd_steerCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &steer_msg) {
		ctrl_cmd_steer_ = steer_msg->drive.steering_angle;
		ctrl_cmd_steer_ = cur_course_ + (ctrl_cmd_steer_*M_PI)/180;
	}
	
	void platformViz(){
		nav_msgs::Odometry course_viz;
		nav_msgs::Odometry ctrl_steer_viz;

		course_viz.pose.pose.position.x = temp_pose_.x;
		course_viz.pose.pose.position.y = temp_pose_.y;
		course_viz.pose.pose.orientation.w = 1.0;

		course_viz.twist.twist.angular.z = cur_course_;
		
		ctrl_steer_viz.pose.pose.position.x = temp_pose_.x;
		ctrl_steer_viz.pose.pose.position.y = temp_pose_.y;
		ctrl_steer_viz.pose.pose.orientation.w = 1.0;

		ctrl_steer_viz.twist.twist.angular.z = ctrl_cmd_steer_;

		curcourse_viz_pub_.publish(course_viz);
		ctrl_viz_pub_.publish(ctrl_steer_viz);

		}
	void getWaypoints(void) {
		std::string first_str_buf, second_str_buf, third_str_buf, fourth_str_buf;
		int first_seg_pos, second_seg_pos, third_seg_pos, fourth_seg_pos; 
		geometry_msgs::Point temp_waypoint;

		is_.open(file_path_);
		
		if(!is_.is_open()) {
			ROS_ERROR("CANNOT FIND [%s], PLEASE CHECK WHETHER IT IS CORRECT PATH.", file_path_.c_str());
			ros::shutdown();
		} else {	
			while(!is_.eof()) {
				getline(is_, first_str_buf);
				if(first_str_buf != "") {
					first_seg_pos = first_str_buf.find(",");

					second_str_buf = first_str_buf.substr(first_seg_pos+1);
					second_seg_pos = second_str_buf.find(",");
					temp_waypoint.x = std::stof(second_str_buf.substr(0, second_seg_pos))-waypoint_x_offset_;
				
					third_str_buf = second_str_buf.substr(second_seg_pos+1);
					third_seg_pos = third_str_buf.find(",");
					temp_waypoint.y = std::stof(third_str_buf.substr(0, third_seg_pos))-waypoint_y_offset_;
			
					waypoints_.push_back(temp_waypoint);
				}
			} 
			is_.close();
		
			std::cout<<waypoints_.size()<<" WAYPOINTS SUCCESSFULLY LOADED."<<std::endl;
		}
	}
	void run(void) {
		ros::Rate loop_rate(10);
		
		while(ros::ok()) {
			visualization_msgs::Marker waypoint_list;
			
			waypoint_list.header.frame_id = "map";
			waypoint_list.header.stamp.fromSec(ros::Time::now().toSec());
			waypoint_list.ns = "waypoint";
			waypoint_list.id = 0;
			waypoint_list.action = visualization_msgs::Marker::ADD;
			waypoint_list.pose.orientation.w = 1.0;
			// visualization_msgs::Marker::LINE_STRIP
			// visualization_msgs::Marker::SPHERE_LIST
			waypoint_list.type = visualization_msgs::Marker::SPHERE_LIST;
			waypoint_list.scale.x = 0.1;
			waypoint_list.scale.y = 0.1;
			waypoint_list.scale.z = 0.1;
			waypoint_list.color.r = 1.0f;
			waypoint_list.color.a = 1.0f;	

			for(int i=0;i<waypoints_.size();i++) {
				waypoint_list.points.push_back(waypoints_[i]);
			}	
	
			waypoint_pub_.publish(waypoint_list);
			
			ros::spinOnce();

			loop_rate.sleep();
		}
	}

	void test() {
		}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_visualizer_node");


	TrajectoryVisualizer tv;
	ros::Rate r(10);
	

	  while (ros::ok()) {
    		tv.platformViz(); 
    		r.sleep();

  }
	
}
