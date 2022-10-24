#pragma once

#include <deque>
#include <functional>

#include <Eigen/Core>
#include <geometry/xform.h>

#include "ekf/state.h"
#include "ekf/meas.h"
#include "common_cpp/common.h"
#include "common_cpp/logger.h"

/// TO ADD A NEW MEASUREMENT
/// Add a new Meas type to the meas.h header file and meas.cpp
/// Add a new callback like mocapCallback()...
/// Add a new update function like mocapUpdate()...
/// Add new cases to the update function
/// Profit.

/// TO ADD A NEW STATE
/// Add an index in the ErrorState and State objects in state.cpp/state.h
/// Make sure the SIZE enums are correct
/// Add relevant Jacobians and Dynamics to measurement update functions and dynamics
/// Profit.



#define CHECK_NAN(mat) \
  if ((mat.array() != mat.array()).any())\
{\
  throw std::runtime_error(#mat " Has NaNs" + std::to_string(__LINE__));\
}

namespace ekf
{

static const Eigen::Vector3d gravity = (Eigen::Vector3d() << 0, 0, 9.80665).finished();

class EKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const dxMat I_BIG;

  EKF();
  ~EKF();

  State& x() { return x_; } // The current state object
  const State& x() const { return x_; }
  dxMat& P() { return P_; } // The current covariance
  const dxMat& P() const { return P_; }

  void initialize(double t, xform::Xformd x0);
  void load(const std::string& filename);
  void initLog(const std::string& filename);

  void run();
  void update(const meas::Base *m);

  bool measUpdate(const Eigen::VectorXd &res, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);
  void propagate(const double& t);
  void dynamics(const State &x, ErrorState &dx, bool calc_jac=false);
  // void errorStateDynamics(const State& x, const ErrorState& dx, const Vector6d& u,
  //                         const Vector6d& eta, ErrorState& dxdot);

  void mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R);

  void mocapUpdate(const meas::Mocap &z);


  void cleanUpMeasurementBuffers();


  void initLog();
  void logState();
  void logCov();
  enum {
    LOG_STATE,
    LOG_COV,
    LOG_GNSS_RES,
    LOG_MOCAP_RES,
    LOG_ZERO_VEL_RES,
    LOG_IMU,
    LOG_LLA,
    LOG_REF,
    NUM_LOGS
  };
  std::vector<std::string> log_names_ {
    "state",
    "cov",
    "gnss_res",
    "mocap_res",
    "zero_vel_res",
    "imu",
    "lla",
    "ref"
  };
  bool enable_log_;
  std::vector<common::Logger*> logs_;
  std::string log_prefix_;

  // Constants
  xform::Xformd x0_;
  Eigen::Vector3d p_b2g_;
  xform::Xformd x_e2I_;
  Eigen::Matrix4d R_zero_vel_;

  // State
  State x_;

  // Matrix Workspace
  dxMat A_;
  dxMat Qx_;
  Matrix6d Qu_;
  dxuMat B_;
  dxuMat K_;
  ErrorState dx_;
  dxMat P_;

  // Distributions
  std::normal_distribution<double> accel_dist_;
  std::normal_distribution<double> accel_dist2_;
  std::normal_distribution<double> omega_dist_;


  // Measurement Buffers
  meas::MeasSet meas_;
  std::deque<meas::Mocap, Eigen::aligned_allocator<meas::Mocap>> mocap_meas_buf_;

  std::default_random_engine rng_;
};

}
