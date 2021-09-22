#include "cvo_server.h"

CVOServer::CVOServer() :
  nh_(), nh_private_("~")
{
  std::string mc_path = ros::package::getPath("crazyflie");
  // nh_private_.param<int>("id", id_, 0);
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", mc_path + "/params/cvo.yaml");

  cvo_service_ = nh_.advertiseService("/cvo", &CVOServer::get_velocity, this);
  subscriber_service_ = nh_.advertiseService("/add_subscriber", &CVOServer::add_subscriber, this);
}

CVOServer::~CVOServer()
{}

void CVOServer::poseCallback(const geometry_msgs::PoseStamped &msg, const std::string &topic)
{
  ;
}


bool CVOServer::add_subscriber(crazyflie::add_subscriber::Request  &req,
         crazyflie::add_subscriber::Response &res)
{

  mocap_ids_.push_back(req.mocap_id);
  // boost::function<void(const boost::shared_ptr<geometry_msgs::PoseStamped const>&)> callback =
  //   boost::bind(CVOServer::poseCallback, _1, boost::ref(req.mocap_id));
  // boost::shared_ptr<CVOServer> foo_object(boost::make_shared<CVOServer>());
  ros::Subscriber pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(req.mocap_id,
      10,
      boost::bind(CVOServer::poseCallback, _1, req.mocap_id),
      this);

  // mocap_subscribers_.push_back(pose_sub);
  res.success = true;
  return true;
}

bool CVOServer::get_velocity(crazyflie::cvo::Request  &req,
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

  // ros::ServiceServer cvo_service = n.advertiseService("/cvo", add);
  // ros::ServiceServer sub_service = n.advertiseService("/add_subscriber", add_subscriber);
  ROS_INFO("CVO Server Ready.");
  ros::spin();

  return 0;
}
