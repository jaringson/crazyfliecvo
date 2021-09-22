#ifndef CVO_SERVER_H
#define CVO_SERVER_H

#include "ros/ros.h"
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>

// #include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include "crazyflie/cvo.h"
#include "crazyflie/add_subscriber.h"

#include "collision_vo/collision_vo.h"


#include <map>
#include <string>


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


  CollisionVO quadCVO_;
  std::map<std::string, std::vector<Vec3d>> allPositions_;
  std::map<std::string, std::vector<Vec3d>> allVelocities_;
  std::vector<std::string> mocap_ids_;


  bool get_velocity(crazyflie::cvo::Request  &req,
          crazyflie::cvo::Response &res);
  bool add_subscriber(crazyflie::add_subscriber::Request  &req,
          crazyflie::add_subscriber::Response &res);
  void poseCallback(const geometry_msgs::PoseStamped &msg, const std::string &topic);
};

#endif //CVO_SERVER_H
