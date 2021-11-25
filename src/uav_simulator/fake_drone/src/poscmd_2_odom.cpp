#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber _cmd_sub;
ros::Publisher  _odom_pub;

ackermann_msgs::AckermannDriveStamped _cmd;
nav_msgs::Odometry last_odom;
// quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y, _init_z;
double wheelbase;
double dt = 0.01;

bool rcv_cmd = false;
void rcvPosCmdCallBack(const ackermann_msgs::AckermannDriveStamped cmd)
{	
	rcv_cmd = true;
	_cmd    = cmd;
}

void normyaw(double& y)
{
  if (y>M_PI)
  {
    y-=2*M_PI;
  }
  else if (y<-M_PI)
  {
    y+=2*M_PI;
  }
}

void pubOdom()
{	
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";

	// if(rcv_cmd)
	// {
	//     odom.pose.pose.position.x = _cmd.position.x;
	//     odom.pose.pose.position.y = _cmd.position.y;
	//     odom.pose.pose.position.z = 0;

	// 	Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
	// 	Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
	// 	Eigen::Vector3d zC(0, 0, 1);
	// 	Eigen::Matrix3d R;
	// 	R.col(0) = xC;
	// 	R.col(1) = yC;
	// 	R.col(2) = zC;
	// 	Eigen::Quaterniond q(R);
	//     odom.pose.pose.orientation.w = q.w();
	//     odom.pose.pose.orientation.x = q.x();
	//     odom.pose.pose.orientation.y = q.y();
	//     odom.pose.pose.orientation.z = q.z();

	//     odom.twist.twist.linear.x = _cmd.velocity.x;
	//     odom.twist.twist.linear.y = _cmd.velocity.y;
	//     odom.twist.twist.linear.z = 0;

	//     odom.twist.twist.angular.x = _cmd.acceleration.x;
	//     odom.twist.twist.angular.y = _cmd.acceleration.y;
	//     odom.twist.twist.angular.z = 0;
	// }
	if(rcv_cmd)
	{
		float v = _cmd.drive.speed;
		float delta = _cmd.drive.steering_angle;
		Eigen::Quaterniond q(last_odom.pose.pose.orientation.w,
			last_odom.pose.pose.orientation.x,
			last_odom.pose.pose.orientation.y,
			last_odom.pose.pose.orientation.z);
		Eigen::Matrix3d R(q);
		Eigen::Vector2d lvel(last_odom.twist.twist.linear.x,last_odom.twist.twist.linear.y);
		double last_yaw = atan2(R.col(0)[1],R.col(0)[0]);
		double last_x = last_odom.pose.pose.position.x;
		double last_y = last_odom.pose.pose.position.y;
		double last_v = lvel.norm();
		double last_yaw_dot = last_odom.twist.twist.angular.z;
		if (fabs(last_yaw_dot)<1e-4)
		{
			odom.pose.pose.position.x = last_x + last_v*cos(last_yaw)*dt;
			odom.pose.pose.position.y = last_y + last_v*sin(last_yaw)*dt;
			odom.pose.pose.position.z = 0;
			odom.pose.pose.orientation = last_odom.pose.pose.orientation;
		}
		else
		{
			odom.pose.pose.position.x = last_x + last_v/last_yaw_dot*(sin(last_yaw+last_yaw_dot*dt)-sin(last_yaw));
			odom.pose.pose.position.y = last_y + last_v/last_yaw_dot*(cos(last_yaw)-cos(last_yaw+last_yaw_dot*dt));
			odom.pose.pose.position.z = 0;
			double yaw = last_yaw + last_yaw_dot * dt;
			normyaw(yaw);
			Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
			Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
			Eigen::Vector3d zC(0, 0, 1);
			Eigen::Matrix3d R;
			R.col(0) = xC;
			R.col(1) = yC;
			R.col(2) = zC;
			Eigen::Quaterniond q(R);
			odom.pose.pose.orientation.w = q.w();
			odom.pose.pose.orientation.x = q.x();
			odom.pose.pose.orientation.y = q.y();
			odom.pose.pose.orientation.z = q.z();
		}

	   	odom.twist.twist.linear.x = v*cos(last_yaw);
		odom.twist.twist.linear.y = v*sin(last_yaw);
		odom.twist.twist.linear.z = 0;
		odom.twist.twist.angular.x = 0;
		odom.twist.twist.angular.y = 0;
		odom.twist.twist.angular.z = v*tan(delta)/wheelbase;
	}
	else
	{
		// odom.pose.pose.position.x = _init_x;
	    // odom.pose.pose.position.y = _init_y;
	    // odom.pose.pose.position.z = _init_z;

	    // odom.pose.pose.orientation.w = 1;
	    // odom.pose.pose.orientation.x = 0;
	    // odom.pose.pose.orientation.y = 0;
	    // odom.pose.pose.orientation.z = 0;

	    // odom.twist.twist.linear.x = 0.0;
	    // odom.twist.twist.linear.y = 0.0;
	    // odom.twist.twist.linear.z = 0.0;

	    // odom.twist.twist.angular.x = 0.0;
	    // odom.twist.twist.angular.y = 0.0;
	    // odom.twist.twist.angular.z = 0.0;
		odom = last_odom;
	}

	last_odom = odom;
    _odom_pub.publish(odom);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);
    nh.param("car/wheelbase", wheelbase,  0.0);

    _cmd_sub  = nh.subscribe( "command", 1, rcvPosCmdCallBack );
    _odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
	last_odom.header.stamp    = ros::Time::now();
	last_odom.header.frame_id = "world";                      
	last_odom.pose.pose.position.x = _init_x;
	last_odom.pose.pose.position.y = _init_y;
	last_odom.pose.pose.position.z = _init_z;

	last_odom.pose.pose.orientation.w = 1;
	last_odom.pose.pose.orientation.x = 0;
	last_odom.pose.pose.orientation.y = 0;
	last_odom.pose.pose.orientation.z = 0;

	last_odom.twist.twist.linear.x = 0.0;
	last_odom.twist.twist.linear.y = 0.0;
	last_odom.twist.twist.linear.z = 0.0;

	last_odom.twist.twist.angular.x = 0.0;
	last_odom.twist.twist.angular.y = 0.0;
	last_odom.twist.twist.angular.z = 0.0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}