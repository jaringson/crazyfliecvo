#pragma once

#include <Eigen/Core>
#include <geometry/xform.h>


namespace ekf
{


class ErrorState
{
public:
    enum {
        DX = 0,
        DP = 0,
        DQ = 3,
        DV = 6,
        DW = 9,
        DA = 12,
        DJ = 15,
        SIZE = 18
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, SIZE, 1> arr;
    Eigen::Map<Vector6d> x;
    Eigen::Map<Eigen::Vector3d> p;
    Eigen::Map<Eigen::Vector3d> q;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> w;
    Eigen::Map<Eigen::Vector3d> a;
    Eigen::Map<Eigen::Vector3d> j;

    ErrorState();
    ErrorState(const ErrorState& obj);
    ErrorState& operator=(const ErrorState& obj);
    ErrorState operator*(const double& s) const;
    ErrorState operator/(const double& s) const;
    ErrorState& operator*=(const double& s);
    ErrorState operator+(const ErrorState& obj) const;
    ErrorState operator-(const ErrorState& obj) const;
    ErrorState operator+(const Eigen::Matrix<double, SIZE, 1>& obj) const;
    ErrorState operator-(const Eigen::Matrix<double, SIZE, 1>& obj) const;
    ErrorState& operator+=(const Eigen::Matrix<double, SIZE, 1>& obj);
    ErrorState& operator-=(const Eigen::Matrix<double, SIZE, 1>& obj);
    ErrorState& operator+=(const ErrorState& obj);
    ErrorState& operator-=(const ErrorState& obj);

    static ErrorState Random()
    {
        ErrorState x;
        x.arr.setRandom();
        return x;
    }

    static ErrorState Zero()
    {
        ErrorState x;
        x.arr.setZero();
        return x;
    }
};

class State
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
      T = 0,
      X = 1, // for Xform access (p, q)
      P = 1,
      Q = 4,
      V = 8,
      W = 11,
      A = 14,
      J = 17,
      SIZE = 20
  };
  Eigen::Matrix<double, SIZE, 1> arr;

  Eigen::Map<Vector7d> mocap; // mocap measurement at current time

  double& t; // Time of current state
  xform::Xformd x;
  Eigen::Map<Eigen::Vector3d> p;
  quat::Quatd q;
  Eigen::Map<Eigen::Vector3d> v;
  Eigen::Map<Eigen::Vector3d> w;
  Eigen::Map<Eigen::Vector3d> a;
  Eigen::Map<Eigen::Vector3d> j;

  State();
  State(const State& other);
  State& operator=(const State& obj);

  static State Random()
  {
      State x;
      x.arr.setRandom();
      x.x = xform::Xformd::Random();
      return x;
  }

  static State Identity()
  {
      State out;
      out.x = xform::Xformd::Identity();
      out.v.setZero();
      out.w.setZero();
      out.a.setZero();
      out.j.setZero();
      return out;
  }

  State operator+(const ErrorState &delta) const;
  State operator+(const Eigen::Matrix<double, ErrorState::SIZE, 1> &delta) const;
  State& operator+=(const ErrorState &delta);
  State& operator+=(const Eigen::VectorXd& dx);
  ErrorState operator-(const State &x2) const;
};

typedef Eigen::Matrix<double, ErrorState::SIZE, ErrorState::SIZE> dxMat;
typedef Eigen::Matrix<double, ErrorState::SIZE, 1> dxVec;
typedef Eigen::Matrix<double, ErrorState::SIZE, 6> dxuMat;
typedef Eigen::Matrix<double, 6, 6> duMat;




}
