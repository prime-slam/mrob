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

SE3tc::SE3tc(const SE3tc &T) : T_(T.T()){}

SE3tc::SE3tc(const Mat31 &omega, const Mat31 &acc, const matData_t &t) : T_(Mat5::Identity())
{
    Mat61 xi;
    xi.head(3) = omega;
    xi.tail(3) = acc;
    this->Exp(xi,t);
}

SE3tc::SE3tc(const Mat61 &xi, const matData_t &t)
{
    this->Exp(xi,t);
}

SE3tc::SE3tc(const SE3 &T, const matData_t &t): T_(Mat5::Identity())
{
    T_.topLeftCorner<3,3>() << T.R();
    if (t<1e-12)
        return;

    Mat31 translation = T.p();
    T_.block<3,1>(0,3) << translation;
    T_(4,3) = t;
    return;
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

Mat<3,5> SE3tc::T_compact() const
{
    return T_.topLeftCorner<3,5>();
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

void SE3tc::Exp(const Mat61& xi, const matData_t &t)  
{
    Mat5 result(Mat5::Identity());

    Mat31 omega = xi.head(3);
    Mat31 phi = omega * t; // Omega * time
    Mat31 acc = xi.tail(3);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();


    result.block<3,1>(0,3) << integrand_2(omega,t)*acc;
    result.block<3,1>(0,4) << integrand_1(omega,t)*acc;
    
    result(4,3) = t;

    this->T_ = result;
}

Mat61 SE3tc::Ln_velocity() const
{
    Mat61 result;

    Mat3 R = this->R();
    Mat31 v = this->v();
    matData_t delta_t = this->t();

    SO3 tmp(R);
    Mat31 omega = tmp.ln_vee()/delta_t;
    Mat3 jac = inv_integrand_1(omega, delta_t);

    result.head(3) << omega;
    result.tail(3) << jac*v;

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

void SE3tc::regenerate()
{
    Mat61 xi = this->Ln_position();
    this->Exp(xi,this->t());
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
    if (delta_t < 1e-12)
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
