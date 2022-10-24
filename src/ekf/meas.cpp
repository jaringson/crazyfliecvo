#include "ekf/meas.h"

namespace ekf::meas
{

Base::Base()
{
    type = BASE;
    handled = false;
}

std::string Base::Type() const
{
    switch (type)
    {
    case BASE:
        return "Base";
        break;
    case GNSS:
        return "Gnss";
        break;
    case IMU:
        return "Imu";
        break;
    case MOCAP:
        return "Mocap";
        break;
    case ZERO_VEL:
        return "ZeroVel";
        break;
    }
}

bool basecmp(const Base* a, const Base* b)
{
    return a->t < b->t;
}


Mocap::Mocap(double _t, const xform::Xformd &_z, const Matrix6d &_R) :
    z(_z),
    R(_R)
{
    t = _t;
    type = MOCAP;
}

ZeroVel::ZeroVel(double _t)
{
    t = _t;
    type = ZERO_VEL;
}
}
