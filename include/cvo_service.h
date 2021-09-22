#ifndef CVO_SERVICE_H
#define CVO_SERVICE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Core>

#include "collision_vo/collision_vo.h"


using namespace Eigen;

class CVOService
{
public:
  CVOService();
  ~CVOService();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  // ROS Stuff


}

#endif //CVO_SERVICE_H
