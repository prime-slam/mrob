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
 * sim3.cpp
 *
 *  Created on: July 7, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/sim3.hpp"
#include "mrob/SO3.hpp"
#include <cmath>


using namespace mrob;

Sim3::Sim3(const Mat4 &S):
    S_(S)
{
}

Sim3::Sim3(const Mat71 &nu) : S_(Mat4::Identity())
{
    this->Exp(nu);
}

Sim3::Sim3(const Sim3 &S) : S_(S.S())
{
}

const Eigen::Ref<const Mat4> Sim3::S() const
{
    return S_;
}


void Sim3::update_lhs(const Mat71 &dnu)
{
    Sim3 dS(dnu);
    S_ = S_ * dS.S();
}


void Sim3::Exp(const Mat71 &nu)
{
    // Exp(nu) = exp[theta^ rho]   = [exp(theta^)  V*rho]
    //              [0  -lamdba]     [0           exp(-labm)]
    //
    // Where V is different case than in SE3, since the recursive series
    // includes now a power of lambda of is infinite series.
    //
    // V = AI + B theta^ + C theta^2
    // This derivation was following Ethan Eade derivation
    // At (https://ethaneade.com/lie.pdf) plus proper Taylor around singularies.


    // 1) calculate the exponent of the rotation, this can be from SO3
    S_.setZero();
    Mat31 theta = nu.head(3);
    Mat3 theta_hat = hat3(theta);
    SO3 R = SO3(theta);
    S_.topLeftCorner<3,3>() = R.R();

    // 2) calculate the exponent of scale
    matData_t lambda = nu(6);
    S_(3,3) = std::exp(-lambda);

    // 3) calculate the translation part
    // A = (1-exp(-lamb)) / lamb
    matData_t A = 1.0;
    if (lambda > 1e-3)
    {
        A = (1 - std::exp(-lambda)) / lambda;
    }
    else
    {
        // Taylor expation around 0
        A = 1 - lambda/2 + lambda*lambda/6;
    }


    // Next terms require to calculate the compoent 
    // B = alpa * (beta - gamma ) + gamma
    // where each of the subcompoents are (folowing Eade's report)
    //   - alpha = (lambda^2)/ (lambda^2 + o2)
    //   - beta = (exp(-lambda) - 1 + lambda) / lambda^2
    //   - gamma = c2 - lambda * c3
    matData_t o = theta.norm();
    matData_t o2 = theta.squaredNorm();
    matData_t c1, c2, c3, c4;
    if ( o > 1e-5){
        // Standard case with the well-known Rodriguez formula
        c1 = std::sin(o)/o;
        c2 = (1 - std::cos(o))/o2;
        c3 = (1 - c1)/o2;
        c4 = (0.5 - c2)/o2;
    }
    else
    {
        c1 =  1 - o2/6.0;
        c2 = 0.5 - o2/24.0;
        c3 = 1.0/6.0 - o2/120;
        c4 = 0.0417 - 0.0014*o2;
    }
    
    matData_t alpha, beta, gamma;
    matData_t lambda2 = lambda * lambda;
    if (lambda2 + o2 > 1e-8)
    {
        alpha = lambda2/ (lambda2 + o2);
    }
    else
    {
        alpha = 1;
    }

    if (lambda2 > 1e-5)
        beta = (std::exp(-lambda) - 1 + lambda) / lambda2;
    else
        beta = 0.5 - 0.1333*lambda + 0.0417*lambda2; // 0.5-x/6+xx/24

    gamma = c2 - lambda * c3;

    matData_t B = alpha * (beta - gamma) + gamma;


    // C = alpha * (mu - v) + v
    matData_t mu, v;
    if (lambda > 1e-3)
        mu = ( 1- lambda +0.5*lambda2 - exp(-lambda))/lambda2;
    else
        mu = 0.1333*lambda - 0.0417*lambda2;
    
    v = c3 - lambda * c4;

    matData_t C = alpha*(mu - v) + v;

    // V = AI + B theta^ + C theta^2
    Mat3 V;
    V = A * Mat3::Identity() + B * theta_hat + C * theta_hat * theta_hat;

    // Calculate the translation component t = V*pho
    Mat31 pho;
    pho << nu(3), nu(4), nu(5);
    Mat31 t = V*pho;
    S_.topRightCorner<3,1>() = t;
}

void Sim3::regenerate()
{
    SO3 R = SO3(S_.topLeftCorner<3,3>());
    R.regenerate();
    S_.topLeftCorner<3,3>() = R.R();
}

std::string Sim3::toString() const
{
    std::stringstream ss;
    ss << this->S_;
    return ss.str();
}
