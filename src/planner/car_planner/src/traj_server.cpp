#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

ros::Publisher pos_cmd_pub;

// quadrotor_msgs::PositionCommand cmd;
ackermann_msgs::AckermannDriveStamped cmd;
double gain, wheelbase,lfc;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool has_odom = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double t_cur;
bool back_flag = false;
double time_forward_;
double last_yaw_, last_yaw_dot_;
double now_yaw, now_v;
Eigen::Vector3d now_pos;
double max_steer, v_max_, max_kappa;

void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  has_odom = true;
  now_pos[0] = msg->pose.pose.position.x;
  now_pos[1] = msg->pose.pose.position.y;
  now_pos[2] = msg->pose.pose.position.z;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z);
  Eigen::Matrix3d R(q);
  Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
  last_yaw_ = now_yaw;
  now_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  now_v = lvel.norm();
}

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if ((!receive_traj_) || (!has_odom))
    return;

  double speed=0.0, steer=0.0;
  if (!back_flag)
  {
    ros::Time time_now = ros::Time::now();
    t_cur = (time_now - start_time_).toSec();
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());

  double dt = 0.1;
  double t_temp = t_cur;
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    while(t_temp<traj_duration_&&(now_pos-traj_[0].evaluateDeBoorT(t_temp)).norm()<lfc)
    {
      t_temp += dt;
    }

    if (t_temp>=traj_duration_)
    {
      t_temp = traj_duration_;
      ROS_ERROR("STOP SHITTING!!!");
      ROS_INFO("vtemp = %f",traj_[1].evaluateDeBoorT(t_temp).norm()); 
    }

    double v_temp = traj_[1].evaluateDeBoorT(t_temp).norm();
    Eigen::Vector3d dir = traj_[0].evaluateDeBoorT(t_temp) - now_pos;
    double yaw_temp = dir.norm() > 0.01 ? atan2(dir(1), dir(0)) : last_yaw_;

    if (t_temp==traj_duration_)
    {
      v_temp = min(dir.norm()/time_forward_, v_max_);
    }

    if (yaw_temp - now_yaw > M_PI)
    {
      if (yaw_temp - now_yaw - 2 * M_PI < -M_PI_2)
      {
        back_flag = true;
        speed = now_v + gain*(v_max_ - now_v);
        steer = -max_steer;
      }
      else
      {
        back_flag = false;
        speed = now_v + gain*(v_temp - now_v);
        steer = -atan2(2*wheelbase*sin(2*M_PI - yaw_temp + now_yaw)/lfc,1);
      }
    }
    else if (yaw_temp - now_yaw < -M_PI)
    {
      if (yaw_temp - now_yaw+ 2 * M_PI > M_PI_2)
      {
        back_flag = true;
        speed = now_v + gain*(v_max_ - now_v);
        steer = max_steer;
      }
      else
      {
        back_flag = false;
        speed = now_v + gain*(v_temp - now_v);
        steer = atan2(2*wheelbase*sin(2*M_PI + yaw_temp - now_yaw)/lfc,1);
      }
    }
    else
    {
      if (yaw_temp - now_yaw < -M_PI_2)
      {
        back_flag = true;
        speed = now_v + gain*(v_max_ - now_v);
        steer = -max_steer;
      }
      else if (yaw_temp - now_yaw > M_PI_2)
      {
        back_flag = true;
        speed = now_v + gain*(v_max_ - now_v);
        steer = max_steer;
      }
      else
      {
        back_flag = false;
        speed = now_v + gain*(v_temp - now_v);
        steer = atan2(2*wheelbase*sin(yaw_temp - now_yaw)/lfc,1);
      }
    }
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    steer = 0;
    speed = 0;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.drive.speed = min(speed, v_max_);
  cmd.drive.steering_angle = max(-max_steer,min(max_steer, steer));
  pos_cmd_pub.publish(cmd);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber odom_sub_ = nh.subscribe( "odom", 1, rcvOdomCallBack );

  pos_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/position_cmd", 50);
  // pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);]

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/max_steer", max_steer, -1.0);
  nh.param("traj_server/v_max", v_max_, -1.0);
  nh.param("traj_server/gain", gain, -1.0);
  nh.param("traj_server/wheelbase", wheelbase, -1.0);
  nh.param("traj_server/forward_distance",lfc,-1.0);
  max_kappa = tan(max_steer)/lfc;

  last_yaw_ = 0.0;
  now_yaw = now_v = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}