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
#include "mrob/SE3.hpp"

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

    Mat31 phi = xi.head(3) * t; // Omega * time
    Mat31 acc = xi.tail(3);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();

    Mat3 integrand_1 = left_jacobian(phi) * t;
    Mat3 integrand_2 = integrand_1;

    result.block<3,1>(0,3) << integrand_1*acc;
    result.block<3,1>(0,4) << integrand_2*acc;
    
    result(4,3) = t;

    this->T_ = result;
}

Mat61 SE3tc::Ln_velocity() const
{
    Mat61 result;

    Mat3 R = this->R();
    Mat31 v = this->v();

    SO3 tmp(R);
    Mat31 log_R_vee = tmp.ln_vee();
    Mat3 jac = inv_left_jacobian(log_R_vee);

    result.head(3) << log_R_vee;
    result.tail(3) << jac*v;

    return result;
}


Mat61 SE3tc::Ln_position() const
{
    Mat61 result;

    Mat3 R = this->R();
    Mat31 p = this->p();

    SO3 tmp(R);
    Mat31 log_R_vee = tmp.ln_vee();
    Mat3 jac = inv_left_jacobian(log_R_vee);

    result.head(3) << log_R_vee;
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