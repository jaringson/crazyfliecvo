#include "ros/ros.h"
#include <ros/package.h>

// #include <eigen3/Eigen/Core>

#include "crazyflie/cvo.h"
#include "crazyflie/add_subscriber.h"

#include "collision_vo/collision_vo.h"

#include <eigen_conversions/eigen_msg.h>

#include <map>
#include <string>

CollisionVO quadCVO;
std::map<std::string, std::vector<Vec3d>> allPositions;
std::map<std::string, std::vector<Vec3d>> allVelocities;
std::vector<std::string> ids;

bool add_subscriber(crazyflie::add_subscriber::Request  &req,
         crazyflie::add_subscriber::Response &res)
{

  ids.push_back(req.mocap_id);
  res.success = true;
  return true;
}

bool add(crazyflie::cvo::Request  &req,
         crazyflie::cvo::Response &res)
{
  // Eigen::Vector3d av1Xo;
  // tf::pointMsgToEigen(req.pos, av1Xo);
  // // Vec3d av1Xo_ = Eigen::Map<Vec3d>(av1Xo.data());
  //
  // Eigen::Vector3d av1Vo;
  // tf::pointMsgToEigen(req.vel, av1Vo);
  // // Vec3d av1Vo_ = Eigen::Map<Vec3d>(av1Vo.data());
  //
  // Eigen::Vector3d av1VelDes;
  // tf::pointMsgToEigen(req.velDes, av1VelDes);
  // // Vec3d av1VelDes_ = Eigen::Map<Vec3d>(av1VelDes.data());
  //
  // Eigen::VectorXd state;
  // state.resize(10);
  // state.segment<3>(3) = av1Vo;
  //
  // Vec4d input_d1;
  //
  // double dt = req.dt;
  //
  // // for(auto a: req.inRan)
  // std::vector<Vec3d> inRangePos; // = req.inRangePos;
  // std::vector<Vec3d> inRangeVel; // = req.inRangeVel;
  //
  // quadCVO.Ts = 1/30.0;
  // quadCVO.numPointsAdmissible = 10;
  // Eigen::Vector3d velCommand = quadCVO.get_best_vel(Eigen::Map<Vec3d>(av1Xo.data()),
  //                     Eigen::Map<Vec3d>(av1Vo.data()),
  //                     Eigen::Map<Vec3d>(av1VelDes.data()),
  //                     state,
  //                     input_d1,
  //                     dt,
  //                     inRangePos,
  //                     inRangeVel,
  //                     inRangePos,
  //                     inRangeVel);
  // ROS_INFO("sending back response");
  // tf::pointEigenToMsg(velCommand, res.velCommand);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cvo_server");
  ros::NodeHandle n;

  ros::ServiceServer cvo_service = n.advertiseService("/cvo", add);
  ros::ServiceServer sub_service = n.advertiseService("/add_subscriber", add_subscriber);
  ROS_INFO("CVO Ready.");
  ros::spin();

  return 0;
}
