#ifndef CVO_SERVER_H
#define CVO_SERVER_H

#include "ros/ros.h"
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include "crazyflie/cvo.h"
#include "crazyflie/add_subscriber.h"

// #include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include "collision_vo/collision_vo.h"
#include "common_cpp/common.h"
#include "geometry/quat.h"

#include "quad_wp_manager.h"

#include <map>
#include <string>

#include <crazyflie/xhat.h>


using namespace Eigen;

class CVOServer
{
public:
  CVOServer();
  ~CVOServer();

private:
  // ROS Stuff
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer cvo_service_;
  ros::ServiceServer subscriber_service_;
  std::vector<ros::Subscriber> mocap_subscribers_;

  std::string package_path_;


  CollisionVO quadCVO_;
  std::map<std::string, Vec3d> allPositions_;
  // std::map<std::string, Vec3d> allPositions_d1_;
  std::map<std::string, Vec3d> allVelocities_;
  std::map<std::string, quat::Quat<double>> allQuats_;
  std::map<std::string, double> allTimes_;
  std::vector<std::string> mocap_ids_;

  double sigma_{0.05};

  double range_;

  std::map<std::string, WPManager> allWPManagers;

  bool get_velocity(crazyflie::cvo::Request  &req,
          crazyflie::cvo::Response &res);
  bool add_subscriber(crazyflie::add_subscriber::Request  &req,
          crazyflie::add_subscriber::Response &res);
  void poseCallback(boost::shared_ptr<crazyflie::xhat const>, const std::string &topic);
};

#endif //CVO_SERVER_H
