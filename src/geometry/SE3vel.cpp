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
 * SE3vel.cpp
 *
 *  Created on: June 12, 2021
 *      Author: Aleksei Panchenko
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/SE3vel.hpp"
#include "mrob/SE3.hpp"

#include <iostream>

using namespace mrob;

SE3vel::SE3vel(const Mat5 &T) : T_(T) {}

SE3vel::SE3vel(const SE3vel &T) : T_(T.T()){}

SE3vel::SE3vel(const Mat91 &xi)
{
    this->Exp(xi);
}
 
Mat31 SE3vel::t() const
{
    return T_.block<3,1>(0,3);
}

Mat31 SE3vel::v() const
{
    return T_.topRightCorner<3,1>();
}

Mat3 SE3vel::R() const
{
    return T_.topLeftCorner<3,3>();
}

Mat5 SE3vel::T(void) const
{
    return this->T_;
}

Mat<3,5> SE3vel::T_compact() const
{
    return T_.topLeftCorner<3,5>();
}

SE3vel SE3vel::inv(void) const
{
    Mat5 inv(Mat5::Zero());
    Mat3 R = this->R();
    R.transposeInPlace();

    inv << R, -R*this->t(), -R*this->v(),
            0,0,0,1,0,
            0,0,0,0,1;

    return SE3vel(inv);
}

SE3vel operator*(const SE3vel& lhs, const SE3vel& rhs)
{
    return SE3vel(Mat5(lhs.T()*rhs.T()));
}

Mat9 SE3vel::adj() const
{
    Mat9 res(Mat9::Zero());
    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 t = this->t();

    res.block<3,3>(0,0) = R;
    res.block<3,3>(3,3) = R;
    res.block<3,3>(6,6) = R;

    res.block<3,3>(3,0) = hat3(t)*R;
    res.block<3,3>(6,0) = hat3(v)*R;

    return res;
}

std::ostream& operator<<(std::ostream &os, const SE3vel& obj)
    {
        os << obj.T();
        return os;
    }


Mat5 hat9(const Mat91 &xi)
{
    Mat5 result(Mat5::Zero());

    result <<    0, -xi(2),  xi(1), xi(3), xi(6),
             xi(2),      0, -xi(0), xi(4), xi(7),
            -xi(1),  xi(0),      0, xi(5), xi(8),
                 0,      0,      0,     0,     0,
                 0,      0,      0,     0,     0;

    return result;   
}


/**
 * Vee operator (v), the inverse of hat
 */
Mat91 vee9(const Mat4 &xi_hat)
{
    Mat91 result(Mat91::Zero());

    result << xi_hat(2,1), xi_hat(0,2), xi_hat(1,0),
              xi_hat(0,3), xi_hat(1,3), xi_hat(2,3),
              xi_hat(0,4), xi_hat(1,4), xi_hat(2,4);

    return result;
}



void SE3vel::Exp(const Mat91& xi)  
{
    Mat5 result(Mat5::Identity());

    Mat31 phi = xi.head(3);
    Mat31 v = xi.segment<3>(3);
    Mat31 t = xi.tail(3);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();

    Mat3 jac = left_jacobian(phi);

    result.block<3,1>(0,3) << jac*v;
    result.block<3,1>(0,4) << jac*t;

    this->T_ = result;
}

Mat91 SE3vel::Ln() const
{
    Mat91 result;

    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 t = this->t();

    SO3 tmp(R);
    Mat31 log_R_vee = tmp.ln_vee();
    Mat3 jac = inv_left_jacobian(log_R_vee);

    result.head(3) << log_R_vee;
    result.segment<3>(3) << jac*v;
    result.tail(3) << jac*t;

    return result;
}

void SE3vel::regenerate()
{
    Mat91 xi = this->Ln();
    this->Exp(xi);
}
