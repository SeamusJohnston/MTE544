//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

const double x_bound = 1;
const double y_bound = 1;
int vertex = 1;
double X = 0;
double Y = 0;
double Yaw = 0;

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	X = msg->pose.pose.position.x; // Robot X psotition
	Y = msg->pose.pose.position.y; // Robot Y psotition
 	Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_square_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		ROS_INFO("Travelling to vertex: %i, x: %f, y: %f, yaw: %f", vertex, X, Y, Yaw);
		vel.linear.x = 0;
		vel.angular.z = 0;
		if (vertex == 1)
		{
			if (X < x_bound)
			{
				vel.linear.x = 0.2;
			}
			else if (Yaw < M_PI/2)
			{
				vel.angular.z = 0.2;
			}
			else
			{
				vertex++;
			}
		}
		else if (vertex == 2)
		{
			if (Y < y_bound)
			{
				vel.linear.x = 0.2;
			}
			else if (Yaw > 0 && Yaw < M_PI)
			{
				vel.angular.z = 0.2;
			}
			else
			{
				vertex++;
			}
		}
		else if (vertex == 3)
		{
			if (X > 0)
			{
				vel.linear.x = 0.2;
			}
			else if (Yaw < -M_PI/2)
			{
				vel.angular.z = 0.2;
			}
			else
			{
				vertex++;
			}
		}
		else
		{
			if (Y > 0)
			{
				vel.linear.x = 0.2;
			}
			else if (Yaw < 0)
			{
				vel.angular.z = 0.2;
			}
			else
			{
				vertex = 1;
			}
		}

		velocity_publisher.publish(vel);
    }

    return 0;
}
