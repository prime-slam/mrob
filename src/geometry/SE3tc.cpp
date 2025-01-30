/* Copyright (c) 2022, Gonzalo Ferrer
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * SE3tc.cpp
 *
 *  Created on: September 9, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/SE3tc.hpp"
#include <Eigen/LU>

#include <iostream>

using namespace mrob;

SE3tc::SE3tc(const Mat5 &T) : T_(T) {}

SE3tc::SE3tc(const Mat5 &T, const matData_t &t) : T_(T)
{
    this->set_time(t);
}

SE3tc::SE3tc(const SE3tc &T) : T_(T.T()){}

SE3tc::SE3tc(const Mat31 &omega, const Mat31 &acc, const matData_t &t) : T_(Mat5::Identity())
{
    Mat91 xi = Mat91::Zero();
    xi.head(3) = omega;
    xi.tail(3) = acc;
    this->Exp(xi,t);
}

SE3tc::SE3tc(const Mat91 &xi, const matData_t &t): T_(Mat5::Identity())
{
    // xi = [omega, vel, acc]
    this->Exp(xi,t);
}

SE3tc::SE3tc(const SE3 &T, const matData_t &t): T_(Mat5::Identity())
{
    T_.topLeftCorner<3,3>() << T.R();

    Mat31 translation = T.p();
    T_.block<3,1>(0,3) = translation;
    this->set_time(t);
    return;
}

SE3tc::SE3tc(const SE3vel &T, const matData_t &t): T_(T.T())
{
    this->set_time(t);
}

Mat31 SE3tc::p() const
{
    return T_.block<3,1>(0,3);
}

Mat31 SE3tc::v() const
{
    return T_.topRightCorner<3,1>();
}

Mat3 SE3tc::R() const
{
    return T_.topLeftCorner<3,3>();
}

Mat5 SE3tc::T(void) const
{
    return this->T_;
}

Mat4 SE3tc::T_SE3(void) const
{
    return T_.topLeftCorner<4,4>();
}

matData_t SE3tc::t(void) const
{
    return this->T_(4,3);//time
}

void SE3tc::set_time(const matData_t &time_stamp)
{
    T_(4,3) = time_stamp;
}

Mat<3,5> SE3tc::T_compact() const
{
    return T_.topLeftCorner<3,5>();
}

SE3vel SE3tc::vel() const
{
    return SE3vel(*this);
}

SE3tc SE3tc::operator*(const SE3tc& rhs)
{
    return SE3tc(Mat5(T_*rhs.T()));
}

SE3tc& SE3tc::operator=(const SE3tc& rhs)
{
    // check for self assignment TODO
    if (this == &rhs)
        return *this;
    T_ = rhs.T();
    return *this;
}

void SE3tc::Exp(const Mat91& xi, const matData_t &t)  
{
    Mat5 result(Mat5::Identity());

    Mat31 omega = xi.head(3);
    Mat31 phi = omega * t;
    Mat31 vel = xi.segment<3>(3);
    Mat31 acc = xi.tail(3);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();

    Mat3 integ1;
    integ1 << integrand_1(omega,t);
    result.block<3,1>(0,3) << integ1*vel + integrand_2(omega,t)*acc;
    result.block<3,1>(0,4) << integ1*acc;
    
    result(4,3) = t;

    this->T_ = result;
}

Mat91 SE3tc::Ln() const
{
    Mat91 result(Mat91::Zero());

    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 p = this->p();
    matData_t delta_t = this->t();
    if ( std::fabs(delta_t) < 1e-12)
        return result;

    SO3 tmp(R);
    Mat31 omega = tmp.ln_vee()/delta_t;
    Mat3 inv_int_1 = inv_integrand_1(omega, delta_t);
    Mat3 int_2 = integrand_2(omega, delta_t);

    result.head(3) << omega;
    result.segment<3>(3) << inv_int_1*p - inv_int_1 * int_2 * inv_int_1 * v;
    result.tail(3) << inv_int_1*v;

    return result;
}


Mat61 SE3tc::Ln_position() const
{
    Mat61 result;

    Mat3 R = this->R();
    Mat31 p = this->p();
    matData_t delta_t = this->t();

    SO3 tmp(R);
    Mat31 omega = tmp.ln_vee()/delta_t;
    Mat3 jac = inv_integrand_2(omega, delta_t);

    result.head(3) << omega;
    result.tail(3) << jac*p;

    return result;
}


SE3tc SE3tc::inv(void) const
{
    Mat5 inv;
    Mat3 R = this->R();
    R.transposeInPlace();
    inv << R, R * ( -this->p() + this->t()*this->v()), -R*this->v(),
           0,0,0,1,0, 
           0,0,0,-this->t(),1;
    return SE3tc(inv);

}


void SE3tc::regenerate()
{
    Mat91 xi = this->Ln();
    this->Exp(xi,this->t());
}


Mat<10,10> SE3tc::adj() const
{
    Mat<10,10> res(Mat<10,10>::Zero());
    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 p = this->p();
    matData_t t = this->t();


    res.block<3,3>(0,0) = R;
    res.block<3,3>(3,3) = R;
    res.block<3,3>(6,6) = R;

    res.block<3,3>(3,0) = hat3(p-t*v)*R;
    res.block<3,3>(6,0) = hat3(v)*R;

    res.block<3,3>(3,6) = -t*R;
    res.block<3,1>(3,9) = v;
    res(9,9) = 1.0;
    return res;
}

Mat<9,9> SE3tc::adj_vel() const
{
    Mat<9,9> res(Mat<9,9>::Zero());
    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 p = this->p();
    matData_t t = this->t();


    res.block<3,3>(0,0) = R;
    res.block<3,3>(3,3) = R;
    res.block<3,3>(6,6) = R;

    res.block<3,3>(3,0) = hat3(p-t*v)*R;
    res.block<3,3>(6,0) = hat3(v)*R;

    res.block<3,3>(3,6) = -t*R;
    return res;
}

void SE3tc::print() const
{
    std::cout << T_ << std::endl;
}


std::ostream& SE3tc::operator<<(std::ostream &os)
{
    os << T_;
    return os;
}

std::string SE3tc::toString() const
{
    std::stringstream ss;
    ss << this->T_;
    return ss.str();
}

Mat3 mrob::integrand_1(const Mat31 &omega, const matData_t &delta_t)
{

    return left_jacobian(omega*delta_t) * delta_t;
}

Mat3 mrob::integrand_2(const Mat31 &omega, const matData_t &delta_t)
{
    Mat3 result = Mat3::Identity()*0.5;
    Mat3 phi_hat = hat3(omega*delta_t);
    if (fabs(delta_t) < 1e-12)
        return result;
    double o = omega.norm();
    double o2 = omega.squaredNorm();
    // If rotation is not zero
    matData_t c3, c4;
    if ( o > 1e-3){
        c3 = (o - std::sin(o))/o2/o;
        c4 = (std::cos(o) - 1.0 + 0.5*o2)/o2/o2;
    }
    else
    {
        // second order Taylor (first order is zero since this is an even function)
        c3 = 1.0/6.0 - o2/120;
        // Second order Taylor
        c4 = 1.0/24.0 - o2/720;
    }
    result += c3*phi_hat + c4*phi_hat*phi_hat;


    return result * (delta_t * delta_t);
}

Mat3 mrob::inv_integrand_1(const Mat31 &omega, const matData_t &delta_t)
{

    return inv_left_jacobian(omega*delta_t) / delta_t;
}

Mat3 mrob::inv_integrand_2(const Mat31 &omega, const matData_t &delta_t)
{
    Mat3 integrand2 = integrand_2(omega, delta_t);
    // there is a closed form, for now we go with brute force inverse.
    return integrand2.inverse();
}


Mat6 mrob::mapping_se3_to_se3tc(const Mat31 &omega, const matData_t &delta_t)
{
    Mat6 result = Mat6::Zero();
    result.topLeftCorner<3,3>() = Mat3::Identity()/delta_t;
    result.bottomRightCorner<3,3>() = inv_integrand_2(omega,delta_t) * left_jacobian(omega*delta_t);
    return result;
}
