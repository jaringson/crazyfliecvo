#include "ekf/ekf_ros.h"
// #include "roscopter_utils/yaml.h"
// #include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace ekf
{

EKF_ROS::EKF_ROS() :
  nh_(), nh_private_("~")
{
}

EKF_ROS::~EKF_ROS()
{}

void EKF_ROS::initROS()
{
  std::string mc_path = ros::package::getPath("crazyflie");
  nh_private_.param<int>("id", id_, 0);
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", mc_path + "/params/ekf_gains.yaml");

  pose_sub_ = nh_.subscribe("/cf"+std::to_string(id_)+"_enu", 10, &EKF_ROS::poseCallback, this);

  odometry_pub_ = nh_.advertise<crazyflie::xhat>("/xhat"+std::to_string(id_),1);

  init(parameter_filename);
}

void EKF_ROS::init(const std::string &param_file)
{
  ekf_.load(param_file);

  double pos_stdev, att_stdev, theta_stdev;
  common::get_yaml_node("position_noise_stdev", param_file, pos_stdev);
  common::get_yaml_node("attitude_noise_stdev", param_file, att_stdev);
  common::get_yaml_node("theta_noise_stdev", param_file, theta_stdev);
  mocap_R_ << pos_stdev * pos_stdev * I_3x3,   Matrix3d::Zero(),
      Matrix3d::Zero(),   att_stdev * att_stdev * I_3x3;

  mocap_R_(4,4) = theta_stdev * theta_stdev;
  start_time_ = ros::Time::now();
  std::cout << "Here in init" << std::endl;
}


void EKF_ROS::poseCallback(const geometry_msgs::PoseStamped &msg)
{
  // if (start_time_.sec == 0)
  // {
    // ekf_.initialize(msg->header.stamp.toSec());
    // start_time_ = msg->header.stamp;
  // }
  // std::cout << "-------------------------------------------------------Callback" << std::endl;

  z_.arr_ << msg.pose.position.x,
          msg.pose.position.y,
          msg.pose.position.z,
          msg.pose.orientation.w,
          msg.pose.orientation.x,
          msg.pose.orientation.y,
          msg.pose.orientation.z;
  // std::cout <<  msg.pose.orientation.w << " " <<  msg.pose.orientation.x << " " <<  msg.pose.orientation.y << " " <<  msg.pose.orientation.z << "\n";
  // std::cout << z.q().w() << " " << z.q().x() << " " << z.q().y() << " " << z.q().z() << std::endl;
  // std::cout << "roll " << z.q().roll()*180.0/M_PI << std::endl;
  // std::cout << "ptich " << z.q().pitch()*180.0/M_PI << std::endl;
  // std::cout << "yaw " << z.q().yaw()*180.0/M_PI << std::endl<< std::endl;

  // double t = (ros::Time::now() - msg->header.stamp).toSec();
  mocapCallback(z_);

}


void EKF_ROS::mocapCallback(const xform::Xformd &z)
{

  double t = (ros::Time::now() - start_time_).toSec();
  ekf_.propagate(t);
  // std::cout << "Message time1: " << t << std::endl;
  ekf_.mocapCallback(t, z, mocap_R_);
  // publish_state();


}

void EKF_ROS::propagate()
{
  // std::cout << "propagate\n";
  double t = (ros::Time::now() - start_time_).toSec();
  // std::cout << "Message time2: " << t << std::endl;
  ekf_.propagate(t);
}

void EKF_ROS::publish_state()
{
  crazyflie::xhat posemsg;
  ekf::State xhat = ekf_.x();
  if((xhat.p.array() != xhat.p.array()).any())
    return;

  // std::cout << "publish\n";
  // std::cout << xhat.q.w() << " " << xhat.q.x() << " " << xhat.q.y() << " " << xhat.q.z() << std::endl;
  // std::cout << "Estimate" << std::endl;
  // std::cout << "roll " << xhat.q.roll()*180.0/M_PI << std::endl;
  // std::cout << "ptich " << xhat.q.pitch()*180.0/M_PI << std::endl;
  // std::cout << "yaw " << xhat.q.yaw()*180.0/M_PI << std::endl<< std::endl;

  posemsg.p[0] = xhat.p[0];
  posemsg.p[1] = xhat.p[1];
  posemsg.p[2] = xhat.p[2];
  posemsg.q[0] = xhat.q.w();
  posemsg.q[1] = xhat.q.x();
  posemsg.q[2] = xhat.q.y();
  posemsg.q[3] = xhat.q.z();
  // posemsg.q[0] = z_.q().w();
  // posemsg.q[1] = z_.q().x();
  // posemsg.q[2] = z_.q().y();
  // posemsg.q[3] = z_.q().z();
  posemsg.v[0] = xhat.v[0];
  posemsg.v[1] = xhat.v[1];
  posemsg.v[2] = xhat.v[2];
  posemsg.a[0] = xhat.a[0];
  posemsg.a[1] = xhat.a[1];
  posemsg.a[2] = xhat.a[2];
  posemsg.w[0] = xhat.w[0];
  posemsg.w[1] = xhat.w[1];
  posemsg.w[2] = xhat.w[2];
  // std::cout << posemsg << std::endl;
  odometry_pub_.publish(posemsg);
}


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_ros");

    ros::NodeHandle nh;

    ekf::EKF_ROS ekf_ros;
    ekf_ros.initROS();

    ros::Rate loop_rate(100);

    ros::Time time;
    ros::Time time_d1 = ros::Time::now();

    while(ros::ok)
    {
      ros::spinOnce();
      loop_rate.sleep();

      ekf_ros.propagate();

      time = ros::Time::now();
      // if((time-time_d1).toSec()>1.0/30.0)
      ekf_ros.publish_state();
    }

    return 0;
}
