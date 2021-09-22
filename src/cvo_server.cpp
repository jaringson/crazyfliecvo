#include "cvo_server.h"

CVOServer::CVOServer() :
  nh_(), nh_private_("~")
{
  std::string mc_path = ros::package::getPath("crazyflie");
  // nh_private_.param<int>("id", id_, 0);
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", mc_path + "/params/cvo.yaml");

  cvo_service_ = nh_.advertiseService("/cvo", &CVOServer::get_velocity, this);
  subscriber_service_ = nh_.advertiseService("/add_subscriber", &CVOServer::add_subscriber, this);

  // calc_beta(1/60.0);
}

CVOServer::~CVOServer()
{}

// void CVOServer::calc_beta(double dt)
// {
//   beta_ = (2.0*sigma_-dt)/(2.0*sigma_+dt);
// }

void CVOServer::poseCallback(boost::shared_ptr<geometry_msgs::PoseStamped const> msg, const std::string &mocap_id)
{

  double timeNow = ros::Time::now().toSec();
  double dt;
  if(allTimes_.find(mocap_id) == allTimes_.end())
    dt = 1/60.0;
  else
    dt = timeNow - allTimes_[mocap_id];
  allTimes_[mocap_id] = timeNow;

  Vec3d positionD1 = allPositions_[mocap_id];
  Vec3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  Vec3d velocity = allVelocities_[mocap_id];
  double beta = (2.0*sigma_-dt)/(2.0*sigma_+dt);
  velocity = beta * velocity + (1.0-beta)* (position - positionD1) / dt;

  allPositions_[mocap_id] = position;
  allVelocities_[mocap_id] = velocity;
}


bool CVOServer::add_subscriber(crazyflie::add_subscriber::Request  &req,
         crazyflie::add_subscriber::Response &res)
{

  mocap_ids_.push_back(req.mocap_id);

  ros::Subscriber pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(req.mocap_id,
      10,
      boost::bind(&CVOServer::poseCallback, this, _1, req.mocap_id));

  mocap_subscribers_.push_back(pose_sub);
  res.success = true;
  return true;
}

bool CVOServer::get_velocity(crazyflie::cvo::Request  &req,
         crazyflie::cvo::Response &res)
{
  Vec3d av1Xo = allPositions_[req.mocap_id];
  Vec3d av1Vo = allVelocities_[req.mocap_id];


  Eigen::Vector3d av1VelDes;
  tf::pointMsgToEigen(req.velDes, av1VelDes);
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
      inRangePos.push_back(x.second);
      inRangeVel.push_back(allVelocities_[x.first]);
    }
  }

  quadCVO_.Ts = dt;
  quadCVO_.numPointsAdmissible = 10;
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
  ROS_INFO("sending back response");
  tf::pointEigenToMsg(velCommand, res.velCommand);
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
