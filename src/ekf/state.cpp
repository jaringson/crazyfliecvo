#include "ekf/state.h"

using namespace Eigen;
using namespace xform;
using namespace quat;
using namespace std;

namespace ekf
{

ErrorState::ErrorState() :
    x(arr.data()),
    p(arr.data()),
    q(arr.data()+3),
    v(arr.data()+6),
    w(arr.data()+9),
    a(arr.data()+12),
    j(arr.data()+15)
{
    arr.setConstant(NAN);
}

ErrorState::ErrorState(const ErrorState& other) :
    ErrorState()
{
    arr = other.arr;
}

ErrorState& ErrorState::operator=(const ErrorState& obj)
{
    arr = obj.arr;
    return *this;
}

ErrorState ErrorState::operator* (const double& s) const
{
    ErrorState out;
    out.arr = s * arr;
    return out;
}

ErrorState ErrorState::operator/ (const double& s) const
{
    ErrorState out;
    out.arr = arr/s;
    return out;
}

ErrorState& ErrorState::operator*= (const double& s)
{
    arr = s * arr;
    return *this;
}

ErrorState ErrorState::operator+ (const ErrorState& obj) const
{
    ErrorState out;
    out.arr = obj.arr + arr;
    return out;
}

ErrorState ErrorState::operator+ (const Matrix<double, SIZE, 1>& v) const
{
    ErrorState out;
    out.arr = v + arr;
    return out;
}

ErrorState& ErrorState::operator+= (const Matrix<double, SIZE, 1>& v)
{
    arr += arr;
    return *this;
}

ErrorState& ErrorState::operator+= (const ErrorState& obj)
{
    arr = obj.arr + arr;
    return *this;
}
ErrorState ErrorState::operator- (const ErrorState& obj) const
{
    ErrorState out;
    out.arr = arr - obj.arr;
    return out;
}

ErrorState ErrorState::operator- (const Matrix<double, SIZE, 1>& v) const
{
    ErrorState out;
    out.arr = arr - v;
    return out;
}

ErrorState& ErrorState::operator-= (const Matrix<double, SIZE, 1>& v)
{
    arr -= arr;
    return *this;
}

ErrorState& ErrorState::operator-= (const ErrorState& obj)
{
    arr = arr - obj.arr;
    return *this;
}

State::State() :
    t(*arr.data()),
    x(arr.data()+1),
    mocap(arr.data()+1),
    p(arr.data()+1),
    q(arr.data()+4),
    v(arr.data()+8),
    w(arr.data()+11),
    a(arr.data()+14),
    j(arr.data()+17)
{
#ifndef NDEBUG
    // to help with tracking down uninitialized memory, in debug mode fill with nans
    arr.setConstant(NAN);
#endif
}

State::State(const State &other) :
    State()
{
    arr = other.arr;
}

State& State::operator= (const State& other)
{
    arr = other.arr;
    return *this;
}

State State::operator+(const ErrorState& dx) const
{
    State xp;
    xp.p = p + dx.p;
    xp.q = q + dx.q;
    xp.v = v + dx.v;
    xp.w = w + dx.w;
    xp.a = a + dx.a;
    xp.j = j + dx.j;
    return xp;
}

State State::operator+(const Matrix<double, ErrorState::SIZE, 1>& dx) const
{
    State xp;
    xp.p = p + dx.segment<3>(ErrorState::DP);
    xp.q = q + dx.segment<3>(ErrorState::DQ);
    xp.v = v + dx.segment<3>(ErrorState::DV);
    xp.w = w + dx.segment<3>(ErrorState::DW);
    xp.a = a + dx.segment<3>(ErrorState::DA);
    xp.j = j + dx.segment<3>(ErrorState::DJ);
    return xp;
}

State& State::operator+=(const VectorXd& dx)
{
    p += dx.segment<3>(ErrorState::DP);
    q += dx.segment<3>(ErrorState::DQ);
    v += dx.segment<3>(ErrorState::DV);
    w += dx.segment<3>(ErrorState::DW);
    a += dx.segment<3>(ErrorState::DA);
    j += dx.segment<3>(ErrorState::DJ);

    return *this;
}

State& State::operator+=(const ErrorState& dx)
{
    p = p + dx.p;
    q = q + dx.q;
    v = v + dx.v;
    w = w + dx.w;
    a = a + dx.a;
    j = j + dx.j;

    return *this;
}

ErrorState State:: operator-(const State& dx) const
{
    ErrorState del;
    del.p = p - dx.p;
    del.q = q - dx.q;
    del.v = v - dx.v;
    // del.a = a - dx.a;
    // del.w = w - dx.w;
    return del;
}


}
