#include "ekf/ekf.h"


using namespace Eigen;
using namespace xform;
using namespace quat;
using namespace std;

#define T transpose()

namespace ekf
{

void EKF::dynamics(const State &x, ErrorState &dx, bool calc_jac)
{
    // Vector3d accel = u.head<3>() - x.a;
    // Vector3d omega = u.tail<3>() - x.w;

    // common::randomNormal(dx.a, accel_dist_, rng_);
    // Eigen::Vector2d temp;
    // common::randomNormal(temp, accel_dist2_, rng_);
    // dx.a.segment<2>(1) = temp;
    // common::randomNormal(dx.w, omega_dist_, rng_);
    dx.j.setZero();
    dx.w.setZero();
    // std::cout << "Dyn Accel: " << dx.a << std::endl;
    // std::cout << "Dyn Omega: " << dx.w << std::endl;
    // dx.a += x.a;
    // dx.w += x.w;
    dx.p = x.q.rota(x.v);
    // dx.p = x.v;
    dx.q = x.w;
    dx.v = x.a; // - x.w.cross(x.v); // + x.q.rotp(gravity) - x.w.cross(x.v);
    dx.a = x.j;

    // CHECK_NAN(dx.arr);
    if (calc_jac)
    {
        Matrix3d R = x.q.R();
        typedef ErrorState DX;
        typedef meas::Mocap U;

        A_.setZero();
        B_.setZero();

        A_.block<3,3>(DX::DP, DX::DQ) = -R.T * skew(x.v);
        A_.block<3,3>(DX::DP, DX::DV) = R.T;
        // A_.block<3,3>(DX::DP, DX::DV) = I_3x3;

        A_.block<3,3>(DX::DQ, DX::DQ) = -skew(x.w);
        A_.block<3,3>(DX::DQ, DX::DW) = I_3x3;

        // A_.block<3,3>(DX::DV, DX::DQ) = skew(x.q.rotp(gravity));
        // A_.block<3,3>(DX::DV, DX::DV) = -skew(x.w);
        // A_.block<3,3>(DX::DV, DX::DW) = skew(x.v);
        A_.block<3,3>(DX::DV, DX::DA) = I_3x3;

        A_.block<3,3>(DX::DA, DX::DJ) = I_3x3;

        B_.block<3,3>(DX::DJ, U::J) = I_3x3;
        B_.block<3,3>(DX::DW, U::W) = I_3x3;
        // B_.block<3,3>(DX::DV, U::W) = skew(x.v);

        // std::cout << "v: " << x.v << std::endl;
        // std::cout<< "A: " << A_ << std::endl;
        // std::cout<< "B: " << B_ << std::endl;

        CHECK_NAN(A_); CHECK_NAN(B_);
    }


}

// void EKF::errorStateDynamics(const State& xhat, const ErrorState& xt, const Vector6d& u,
//                              const Vector6d& eta, ErrorState& dxdot)
// {
//     auto eta_a = eta.head<3>();
//     auto eta_w = eta.tail<3>();
//     auto z_a = u.head<3>();
//     auto z_w = u.tail<3>();
//
//     State x = xhat + xt;
//     Vector3d a = z_a - x.a + eta_a;
//     Vector3d ahat = z_a - xhat.a;
//     Vector3d w = z_w - x.w + eta_w;
//     Vector3d what = z_w - xhat.w;
//     dxdot.arr.setZero();
//     dxdot.p = x.q.rota(x.v) - xhat.q.rota(xhat.v);
//     dxdot.q = w - x.q.rotp(xhat.q.rota(what));
//     dxdot.v = x.q.rotp(gravity) + a - w.cross(x.v) - (xhat.q.rotp(gravity) + ahat - what.cross(xhat.v));
// }

}
