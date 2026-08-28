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
 * Sim3.cpp
 *
 *  Created on: Aug 26, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/Sim3.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <Eigen/LU>


using namespace mrob;

Sim3::Sim3(const Mat4 &T) :
        T_(T)
{
}

Sim3::Sim3(const Mat71 &xi) : T_(Mat4::Identity())
{
    //std::cout << "Sim3 MAT31" << std::endl;
    this->exp(xi);
}

Sim3::Sim3(const Sim3 &T): T_(T.T())
{
}

template<typename OtherDerived>
Sim3::Sim3(const Eigen::MatrixBase<OtherDerived>& rhs)  :
    T_(rhs)
{    //std::cout << "Sim3 MAT4" << std::endl;
}


Sim3& Sim3::operator=(const Sim3& rhs)
{
    // check for self assignment TODO
    if (this == &rhs)
        return *this;
    T_ = rhs.T();
    return *this;
}

Sim3 Sim3::operator*(const Sim3& rhs) const
{
    Mat4 res = T_ * rhs.T();
    return Sim3(res);
}


Sim3 Sim3::mul(const Sim3& rhs) const
{
    return (*this) * rhs;
}

void Sim3::update_lhs(const Mat71 &dxi)
{
    Sim3 dT(dxi);
    T_ = dT.T() * T_;
}
void Sim3::update_rhs(const Mat71 &dxi)
{
    Sim3 dT(dxi);
    T_ = T_ * dT.T();
}

Mat71 mrob::vee7(const Mat4 &xi_hat)
{
    Mat71 xi;
    xi << -xi_hat(1,2), xi_hat(0,2), -xi_hat(0,1),
           xi_hat(0,3), xi_hat(1,3), xi_hat(2,3),
           xi_hat(0,0); //it should be the same in 1,1 and 2,2
    return xi;
}

Mat4 mrob::hat7(const Mat71 &xi)
{
    Mat4 xi_hat;
    xi_hat  <<    xi(6), -xi(2),  xi(1), xi(3),
                xi(2),    xi(6), -xi(0), xi(4),
               -xi(1),  xi(0),    xi(6), xi(5),
                    0,      0,      0,    0;
    return xi_hat;
}

void Sim3::exp(const Mat71 &xi)
{
    // Calculating xi = [w, v, sigma]
    Mat31 omega = xi.head<3>();
    Mat31 rho = xi.segment<3>(3);
    matData_t sigma = xi(6);
    matData_t s = std::exp(sigma);
    SO3 rotation(omega);
    Mat3 sR = s*rotation.R();

    // Calculate the closed form of V
    Mat3 V;
    if (s < 1e-10)
        V = left_jacobian(omega);
    else{
        Mat3 K = sigma * Mat3::Identity() + hat3(omega) ;
        V = K.inverse() * (sR - Mat3::Identity());
    }

    // Calculate the translation component t = Vv
    Mat31 t = V*rho;

    // compose the matrix T = [sR, t]
    //this->topLeftCorner<3,3>() = R;
    //this->topRightCorner<3,1>() = t;
    T_  << sR, t,
           0,0,0,1;
}

Mat71 Sim3::ln(void) const
{
    matData_t s = this->s();
    Mat3 sR = this->sR();
    Mat3 R = sR/s;
    Mat31 omega = SO3(R).ln_vee();
    matData_t sigma = std::log(s);

    // calculate v =  (sR-I)/(sigma+omega^) * t
    Mat3 sRI = sR - Mat3::Identity();
    Mat3 Vinv =  sRI.inverse() * (hat3(omega) + sigma * Mat3::Identity());

    // v = V^-1 t
    Mat31 v = Vinv * T_.topRightCorner<3,1>();

    // create a vector containing the components
    Mat71 xi;
    xi.head<3>() = omega;
    xi.segment<3>(3) = v;
    xi(6) = sigma;
    return xi;
}

Mat31 Sim3::transform(const Mat31 & p) const
{
    return this->sR()*p + this->t();
}


MatX Sim3::transform_array(const MatX &P) const
{
    assert(P.cols() == 3 && "Sim3::transformArray: incorrect data structure, it is required an Nx3 input");
    uint_t N = P.rows();
    MatX res(N,3);
    for (uint_t i = 0; i < N; ++i)
        res.row(i) << this->transform(P.row(i)).transpose();
    return res;
}


Sim3 Sim3::inv(void) const
{
    Mat4 inv;
    matData_t s = this->s();
    Mat3 sR = this->sR();
    matData_t inv_s = 1/s/s;
    Mat3 inv_sR;
    inv_sR = inv_s*sR;
    sR.transposeInPlace();
    inv << inv_sR, -inv_sR * this->t(),
           0,0,0,1;
    return Sim3(inv);

}

Mat7 Sim3::adj() const
{
    Mat7 res(Mat7::Zero());
    Mat3 tx = hat3( this->t() );
    res.topLeftCorner<3,3>() << sR();
    res.bottomRightCorner<3,3>() << sR();
    res.bottomLeftCorner<3,3>() << tx*sR();
    return res;
}

//Mat4 Sim3::T() const
const Eigen::Ref<const Mat4> Sim3::T() const
{
    return T_;
}

Mat4& Sim3::ref2T()
{
    return T_;
}

Mat3 Sim3::sR() const
{
    return T_.topLeftCorner<3,3>();
}

Mat31 Sim3::t() const
{
    return T_.topRightCorner<3,1>();
}

Mat31 Sim3::p() const
{
    return T_.topRightCorner<3,1>();
}

matData_t Sim3::s() const
{
    matData_t sq_norm = sR().squaredNorm();
    return std::sqrt(sq_norm/3.0);
}

void Sim3::print(void) const
{
    std::cout << T_ << std::endl;
}


void Sim3::print_lie(void) const
{

    std::cout << this->ln() << std::endl;
}

void Sim3::regenerate()
{
    Mat71 xi = this->ln();
    this->exp(xi);
}

Mat41 Sim3::transform_plane(const Mat41 &pi)
{
    return this->inv().T().transpose() * pi;
}



std::string Sim3::toString() const
{
    std::stringstream ss;
    ss << this->T_;
    return ss.str();
}


