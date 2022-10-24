#include "quad_wp_manager.h"

// namespace quadrotor
// {

WPManager::WPManager() :
  initialized_(false)
{

}

WPManager::~WPManager()
{

}

void WPManager::load(const std::string& filename,
  std::vector<double> loaded_wps,
  const double& max_velocity)
{
  int num_waypoints = std::floor(loaded_wps.size()/4.0);
  waypoints_ = Map<MatrixXd>(loaded_wps.data(), 4, num_waypoints);
  current_waypoint_id_ = 0;
  // std::cout << "Waypoints Loaded: " << waypoints_ << std::endl;

  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  common::get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_);

  Vector3d Kp_diag;
  if (common::get_yaml_eigen("Kp", filename, Kp_diag))
    K_p_ = Kp_diag.asDiagonal();

  if(max_velocity == 0.0)
    common::get_yaml_node("max_vel", filename, max_vel_);
  else
    max_vel_ = max_velocity;
}

Vector3d WPManager::updateWaypointManager(Vector3d position)
{

  // Vector3d positionCommand;
  double psiCommand;

  // std::cout << "pos wp mang: " << position << std::endl;

  // if (!initialized_)
  // {
  //   initialized_ = true;
  //   Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, 0).data());
  //   positionCommand = new_waypoint.segment<3>(0);
  //   psiCommand = quat::Quatd(0, 0, new_waypoint(3));
  // }

  // Find the distance to the desired waypoint
  Vector4d current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_);
  // std::cout << "current_wp: " << current_waypoint << std::endl;
  Vector3d error;
  error = current_waypoint.segment<3>(0) - position;
  // std::cout << "error: " << error << std::endl;
  // error(0) = common::wrapAngle(current_waypoint(0) - xhat_.q.yaw(), 2.0 * M_PI);
  // error(3) = 0;

  if(error.norm() < waypoint_threshold_)
  {
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();
    current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_);
    error = current_waypoint.segment<3>(0) - position;
  }

  Vector3d desiredVelocity = K_p_ * error;

  if(desiredVelocity.norm() > max_vel_)
    desiredVelocity = desiredVelocity.normalized()*max_vel_;



  return desiredVelocity;
}




// }
