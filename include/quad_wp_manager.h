#pragma once

#include "common_cpp/common.h"
// #include "vehicle.h"


using namespace Eigen;


// namespace quadrotor
// {


class WPManager
{

public:
  WPManager();
  ~WPManager();

  void load(const std::string& filename,
    std::vector<double> loaded_wps,
    const double& max_velocity=0);

  Vector3d updateWaypointManager(Vector3d position);

private:

  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ,
    PSI
  };

  // Memory for sharing information between functions
  // vehicle::Stated xhat_; // estimate
  bool initialized_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  double waypoint_velocity_threshold_;

  Matrix3d K_p_; // velocity

  double max_vel_;

};

// }
