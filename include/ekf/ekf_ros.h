#pragma once


#include "ekf.h"
#include "state.h"

#include <mutex>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
// #include <nav_msgs/Odometry.h>
// #include <rosflight_msgs/Status.h>
// #include <rosflight_msgs/GNSS.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <crazyflie/xhat.h>

namespace ekf
{
class EKF_ROS
{
public:

  EKF_ROS();
  ~EKF_ROS();
  void init(const std::string& param_file);
  void initROS();

  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void mocapCallback(const xform::Xformd &z);

  void publish_state();
  void propagate();


private:
  EKF ekf_;

  int id_{0};

  xform::Xformd z_;

  ros::Time last_imu_update_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher odometry_pub_;
  // ros::Subscriber odom_sub_;
  // ros::Subscriber gnss_sub_;
  // ros::Subscriber status_sub_;

  // ros::Publisher bias_pub_;
  // ros::Publisher is_flying_pub_;
  // nav_msgs::Odometry odom_msg_;

  // std::mutex ekf_mtx_;

  // bool truth_init_ = false;
  // bool imu_init_ = false;

  // bool use_odom_;
  // bool use_pose_;

  // bool is_flying_ = false;
  // bool armed_ = false;
  // ros::Time time_took_off_;
  ros::Time start_time_;

  // Vector6d imu_;

  // Matrix6d imu_R_;
  Matrix6d mocap_R_;
  // Eigen::Matrix<double, 1, 1> alt_R_;
};

}
