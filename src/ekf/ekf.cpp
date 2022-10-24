#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace ekf
{

const dxMat EKF::I_BIG = dxMat::Identity();

EKF::EKF()
{

}

EKF::~EKF()
{
  // for (int i = 0; i < NUM_LOGS; i++)
  //   delete logs_[i];
}

void EKF::load(const std::string &filename)
{
  double accel_noise_stdev, accel_noise_stdev2, omega_noise_stdev;
  common::get_yaml_node("accel_noise_stdev", filename, accel_noise_stdev);
  common::get_yaml_node("accel_noise_stdev2", filename, accel_noise_stdev2);
  accel_dist_ = std::normal_distribution<double>(0.0, accel_noise_stdev);
  accel_dist2_ = std::normal_distribution<double>(0.0, accel_noise_stdev2);

  common::get_yaml_node("omega_noise_stdev", filename, omega_noise_stdev);
  omega_dist_ = std::normal_distribution<double>(0.0, omega_noise_stdev);
  // // Constant Parameters
  // get_yaml_eigen("p_b2g", filename, p_b2g_);
  common::get_yaml_eigen_diag("Qx", filename, Qx_);
  common::get_yaml_eigen_diag("P0", filename, P_);

  // get_yaml_diag("R_zero_vel", filename, R_zero_vel_);

  // // Measurement Flags
  // get_yaml_node("enable_out_of_order", filename, enable_out_of_order_);
  // get_yaml_node("use_truth", filename, use_truth_);
  // get_yaml_node("use_gnss", filename, use_gnss_);
  // get_yaml_node("use_alt", filename, use_alt_);
  // get_yaml_node("use_zero_vel", filename, use_zero_vel_);

  // // load initial state
  // Vector3d ref_lla;
  // double ref_heading;
  // get_yaml_eigen("ref_lla", filename, ref_lla);
  // get_yaml_node("ref_heading", filename, ref_heading);
  // ref_lla.head<2>() *= M_PI/180.0; // convert to rad
  // quat::Quatd q_n2I = quat::Quatd::from_euler(0, 0, M_PI/180.0 * ref_heading);
  // xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  // x_e2I_.t() = x_e2n.t();
  // x_e2I_.q() = x_e2n.q() * q_n2I;
  common::get_yaml_eigen("x0", filename, x0_.arr());

  // initLog(filename);
}

// void EKF::initLog(const std::string &filename)
// {
//   get_yaml_node("enable_log", filename, enable_log_);
//   get_yaml_node("log_prefix", filename, log_prefix_);
//
//   std::experimental::filesystem::create_directories(log_prefix_);
//
//   logs_.resize(NUM_LOGS);
//   for (int i = 0; i < NUM_LOGS; i++)
//     logs_[i] = new Logger(log_prefix_ + "/" + log_names_[i] + ".bin");
// }

void EKF::initialize(double t, xform::Xformd x0)
{
  x().t = t;
  x().x = x0;
  // common::randomNormal(x().v, accel_dist_, rng_);
  x().v.setZero();
  // x().v[0] = 3;
  // common::randomNormal(x().a, accel_dist_, rng_);
  // x().a = gravity;
  x().w.setZero();
  x().a.setZero();
  x().j.setZero();
  // common::randomNormal(x().w, omega_dist_, rng_);

  // Initialize random number generator
  int seed = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = std::default_random_engine(seed);
  srand(seed);
}

void EKF::propagate(const double &t)
{
  if (std::isnan(x().t))
  {
    // initialize(t);
    return;
  }

  double dt = t - x().t;
  assert(dt >= 0);
  if (dt < 1e-6)
    return;


  dynamics(x_, dx_, true);

  // std::cout << "\t\tPropagate Before: " << dt << std::endl;
  // std::cout << x_.p << std::endl;
  // std::cout << x_.q << std::endl;
  // std::cout << x_.v << std::endl;
  // std::cout << x_.w << std::endl;
  // std::cout << x_.a << std::endl;
  // std::cout << x_.j << std::endl;


  // do the state propagation
  x_ = x() + dx_ * dt;
  x_.t = t;

  // std::cout << "\t\tPropagate After: " << dt << std::endl;
  // std::cout << x_.p << std::endl;
  // std::cout << x_.q << std::endl;
  // std::cout << x_.v << std::endl;
  // std::cout << x_.w << std::endl;
  // std::cout << x_.a << std::endl;
  // std::cout << x_.j << std::endl;


  // discretize jacobians (first order)
  A_ = I_BIG + A_*dt; // + A_ * A_ * dt * dt / 2.0;
  B_ = B_ * dt; // + A_ * B_ * dt * dt / 2.0;
  CHECK_NAN(P());
  CHECK_NAN(A_);
  CHECK_NAN(B_);
  CHECK_NAN(Qx_);
  // std::cout << "Before P_: " << P_ << std::endl;
  P_ = A_*P()*A_.T + B_*Qu_*B_.T + Qx_*dt*dt; // covariance propagation
  CHECK_NAN(P_);
  // std::cout << "After P_: " << P_ << std::endl;
  // Qu_ = R; // copy because we might need it later.

  // if (enable_log_)
  // {
  //   logs_[LOG_STATE]->logVectors(x().arr, x().q.euler());
  //   logs_[LOG_COV]->log(x().t);
  //   logs_[LOG_COV]->logVectors(P());
  //   logs_[LOG_IMU]->log(t);
  //   logs_[LOG_IMU]->logVectors(imu);
  // }
}

//
// void EKF::update(const meas::Base* m)
// {
//   std::cout << x().t << std::endl;
//   if (!std::isnan(x().t))
//   {
//     propagate(m->t);
//     switch(m->type)
//     {
//     case meas::Base::MOCAP:
//       {
//         const meas::Mocap* z = dynamic_cast<const meas::Mocap*>(m);
//         mocapUpdate(*z);
//         break;
//       }
//     default:
//       break;
//     }
//   }
// }


bool EKF::measUpdate(const VectorXd &res, const MatrixXd &R, const MatrixXd &H)
{
  Qu_ = R;
  // std::cout << "R: " << R << std::endl;
  int size = res.rows();
  auto K = K_.leftCols(size);

  ///TODO: perform covariance gating
  MatrixXd innov = (H*P()*H.T + R).inverse();

  CHECK_NAN(H); CHECK_NAN(R); CHECK_NAN(P());
  K = P() * H.T * innov;
  CHECK_NAN(K);

  ///TODO: Partial Update
  x_ += K * res;

  dxMat ImKH = I_BIG - K*H;

  P() = ImKH*P()*ImKH.T + K*R*K.T;
  // std::cout << "ImKH: " << ImKH << std::endl;

  CHECK_NAN(P());

  return false;
}




void EKF::mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R)
{
  if (std::isnan(x().t))
  {
    std::cout << "Initializing" << std::endl;
    initialize(t,z);
  }
  mocapUpdate(meas::Mocap(t, z, R));

  // if (enable_log_)
  // {
  //   logs_[LOG_REF]->log(t);
  //   logs_[LOG_REF]->logVectors(z.arr(), z.q().euler());
  // }
}



void EKF::mocapUpdate(const meas::Mocap &z)
{
  xform::Xformd zhat = x().x;
  Vector6d r; // = zhat - z.z;
  r.head<3>() = z.z.t() - zhat.t();
  r.tail<3>() = z.z.q() - zhat.q();

  typedef ErrorState E;
  Matrix<double, 6, E::SIZE> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = I_3x3;
  H.block<3,3>(3, E::DQ) = I_3x3;


  /// TODO: Saturate r

  measUpdate(r, z.R, H);

  // if (enable_log_)
  // {
  //   logs_[LOG_MOCAP_RES]->log(z.t);
  //   logs_[LOG_MOCAP_RES]->logMatrix(r, z.z.arr(), zhat.arr());
  // }
}


}
