#include "cvo_server.h"

CVOServer::CVOServer() :
  nh_(), nh_private_("~")
{
  package_path_ = ros::package::getPath("crazyflie");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", package_path_ + "/params/cvo.yaml");

  cvo_service_ = nh_.advertiseService("/cvo", &CVOServer::get_velocity, this);
  subscriber_service_ = nh_.advertiseService("/add_subscriber", &CVOServer::add_subscriber, this);

  double numPointsAdmissible, collisionRadius, range, bufferPower, collision_acceleration;
  common::get_yaml_node("num_points_admissible", parameter_filename, numPointsAdmissible);
  common::get_yaml_node("collision_acceleration", parameter_filename, collision_acceleration);
  common::get_yaml_node("collision_radius", parameter_filename, collisionRadius);
  common::get_yaml_node("range", parameter_filename, range);
  common::get_yaml_node("buffer_power", parameter_filename, bufferPower);

  bool bufferOn;
  common::get_yaml_node("buffer_on", parameter_filename, bufferOn);

  quadCVO_.numPointsAdmissible = numPointsAdmissible;
  quadCVO_.maxAccel = collision_acceleration;
  quadCVO_.collisionRadius = collisionRadius;
  // quadCVO_.range = range;
  range_ = range;

  quadCVO_.bufferPower = bufferPower;
  quadCVO_.bufferOn = bufferOn;
}

CVOServer::~CVOServer()
{}



void CVOServer::poseCallback(boost::shared_ptr<crazyflie::xhat const> msg, const std::string &mocap_id)
{

  double timeNow = ros::Time::now().toSec();
  // double dt;
  // if(allTimes_.find(mocap_id) == allTimes_.end())
  //   dt = 1/60.0;
  // else
  //   dt = timeNow - allTimes_[mocap_id];
  allTimes_[mocap_id] = timeNow;

  Vec3d positionD1 = allPositions_[mocap_id];
  Vec3d position(msg->p[0],
                    msg->p[1],
                    msg->p[2]);
  Vec4d orientation(msg->q[0],
                    msg->q[1],
                    msg->q[2],
                    msg->q[3]);
  quat::Quat<double> quatOrientation(orientation);

  Vec3d velocity(msg->v[0],
                    msg->v[1],
                    msg->v[2]);

  // Vec3d velocity;
  // if(allVelocities_.find(mocap_id) == allVelocities_.end())
  //   velocity.setZero();
  // else
  //   velocity = allVelocities_[mocap_id];
  // double beta = (2.0*sigma_-dt)/(2.0*sigma_+dt);
  // velocity = beta * velocity + (1.0-beta)* (position - positionD1) / dt;

  allPositions_[mocap_id] = position;
  allVelocities_[mocap_id] = velocity;
  allQuats_[mocap_id] = quatOrientation;
}


bool CVOServer::add_subscriber(crazyflie::add_subscriber::Request  &req,
         crazyflie::add_subscriber::Response &res)
{

  mocap_ids_.push_back(req.mocap_id);

  ros::Subscriber pose_sub = nh_.subscribe<crazyflie::xhat>(req.mocap_id,
      10,
      boost::bind(&CVOServer::poseCallback, this, _1, req.mocap_id));

  mocap_subscribers_.push_back(pose_sub);

  WPManager wpManager;
  std::vector<double> waypoints(std::begin(req.waypoints), std::end(req.waypoints));
  wpManager.load(package_path_+"/params/quadrotor1.yaml", waypoints);
  allWPManagers[req.mocap_id] = wpManager;

  res.success = true;
  return true;
}

bool CVOServer::get_velocity(crazyflie::cvo::Request  &req,
         crazyflie::cvo::Response &res)
{
  Vec3d av1Xo = allPositions_[req.mocap_id];
  Vec3d av1Vo = allVelocities_[req.mocap_id];


  Eigen::Vector3d av1VelDes = allWPManagers[req.mocap_id].updateWaypointManager(
              Eigen::Map<Vector3d>(av1Xo.data()));
  // av1VelDes = allQuats_[req.mocap_id].rotp(av1VelDes);
  // tf::pointMsgToEigen(req.velDes, av1VelDes);
  // Vec3d av1VelDes_ = Eigen::Map<Vec3d>(av1VelDes.data());

  Eigen::VectorXd state;
  state.resize(10);
  state.segment<3>(3) = av1Vo;

  Vec4d input_d1;

  double dt = req.dt;

  std::vector<Vec3d> inRangePos; // = req.inRangePos;
  std::vector<Vec3d> inRangeVel; // = req.inRangeVel;
  for (auto const& x : allPositions_)
  {
    if(req.mocap_id != x.first)
    {
      if((av1Xo-x.second).norm() < range_)
      {
        inRangePos.push_back(x.second);
        inRangeVel.push_back(allVelocities_[x.first]);
      }
    }
  }

  quadCVO_.Ts = dt;
  Eigen::Vector3d velCommand = quadCVO_.get_best_vel(av1Xo,
                      av1Vo,
                      Eigen::Map<Vec3d>(av1VelDes.data()),
                      state,
                      input_d1,
                      dt,
                      inRangePos,
                      inRangeVel,
                      inRangePos,
                      inRangeVel);
  tf::pointEigenToMsg(allQuats_[req.mocap_id].rotp(velCommand), res.velCommand);
  // tf::pointEigenToMsg(allQuats_[req.mocap_id].rotp(av1VelDes), res.velCommand);
  // tf::pointEigenToMsg(av1VelDes, res.velCommand);
  // ROS_INFO("Number of in Range: %i", int(inRangePos.size()));
  // std::cout << av1Vo << std::endl << std::endl;
  geometry_msgs::Point position;
  tf::pointEigenToMsg(av1Xo, position);
  geometry_msgs::Quaternion orientation;
  Eigen::Quaterniond quat;
  quat.w() = allQuats_[req.mocap_id].w();
  quat.x() = allQuats_[req.mocap_id].x();
  quat.y() = allQuats_[req.mocap_id].y();
  quat.z() = allQuats_[req.mocap_id].z();
  tf::quaternionEigenToMsg(quat, orientation);
  res.pose.position = position;
  res.pose.orientation = orientation;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cvo_server");
  ros::NodeHandle n;
  CVOServer server;

  // ros::ServiceServer cvo_service = n.advertiseService("/cvo", add);
  // ros::ServiceServer sub_service = n.advertiseService("/add_subscriber", add_subscriber);
  ROS_INFO("CVO Server Ready.");
  ros::spin();

  return 0;
}
