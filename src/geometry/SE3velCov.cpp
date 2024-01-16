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
 * SE3velCov.cpp
 *
 *  Created on: June 12, 2021
 *      Author: Aleksei Panchenko
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/SE3velCov.hpp"
#include <iostream>

using namespace mrob;

SE3velCov::SE3velCov(void)
{
    this->T_ = Mat5::Identity();
    this->covariance = Mat9::Identity();
}

SE3velCov::SE3velCov(const SE3vel &pose, const Mat9 &covariance)
{
    this->T_ = pose.T();
    this->covariance = covariance;
}

Mat9 SE3velCov::getQ(const Mat3 &cov_a, const Mat3 &cov_w, const double dt) const
{
    Mat9 Q(Mat9::Identity());

    Q.topLeftCorner<3, 3>() = cov_w;
    Q.block<3, 3>(3, 3) = cov_a;
    Q.block<3, 3>(6, 3) = 1. / 2. * cov_a * dt;
    Q.block<3, 3>(3, 6) = 1. / 2. * cov_a * dt;
    Q.bottomRightCorner<3, 3>() = 1. / 4. * cov_a * dt * dt;
    Q *= dt * dt;

    return Q;
}

void SE3velCov::compound_2nd_order(const SE3vel &pose_increment, const Mat9 &Q, const double dt)
{
    Mat9 F(Mat9::Identity());
    F.block<3, 3>(6, 3) = dt * Mat3::Identity();

    Mat9 Gamma_inv_adj = pose_increment.inv().adj();

    Mat9 tmp = Gamma_inv_adj * F;

    this->covariance = tmp * this->covariance * tmp.transpose() + Q;

    this->T_ = this->T() * pose_increment.T();
}

void SE3velCov::compound_4th_order(const SE3vel &pose_increment, const Mat9 &Q, const double dt)
{
    Mat9 S_4th = this->get_S_4th(Q); // get_s_4th mus be called exactly before calling compound_2nd
    compound_2nd_order(pose_increment, Q, dt);

    this->covariance += S_4th;
}

Mat9 SE3velCov::get_S_4th(const Mat9 &Q) const
{
    Mat9 S_4th(Mat9::Zero( ));

    Mat9 A_sigma(Mat9::Zero());

    Mat9 sigma = this->covariance;
    Mat3 sigma_phi_phi = sigma.topLeftCorner<3, 3>();
    Mat3 sigma_phi_nu = sigma.block<3, 3>(0, 3);
    Mat3 sigma_nu_phi = sigma_phi_nu.transpose();
    Mat3 sigma_phi_rho = sigma.topRightCorner<3, 3>();
    Mat3 sigma_rho_phi = sigma_phi_rho.transpose();
    Mat3 sigma_nu_nu = sigma.block<3, 3>(3, 3);
    Mat3 sigma_nu_rho = sigma.block<3, 3>(3, 6);
    Mat3 sigma_rho_rho = sigma.bottomRightCorner<3, 3>();

    A_sigma.topLeftCorner<3, 3>() = brackets(sigma_phi_phi);
    A_sigma.block<3, 3>(3, 0) = brackets(sigma_phi_nu.transpose() + sigma_phi_nu);
    A_sigma.block<3, 3>(3, 3) = brackets(sigma_phi_phi);
    A_sigma.bottomLeftCorner<3, 3>() = brackets(sigma_phi_rho.transpose() + sigma_phi_rho);
    A_sigma.bottomRightCorner<3, 3>() = brackets(sigma_phi_phi);

    Mat9 A_q(Mat9::Zero());

    Mat3 q_phi_phi = Q.topLeftCorner<3, 3>();
    Mat3 q_phi_nu = Q.block<3, 3>(0, 3);
    Mat3 q_nu_phi = q_phi_nu.transpose();
    Mat3 q_phi_rho = Q.topRightCorner<3, 3>();
    Mat3 q_rho_phi = q_phi_rho.transpose();
    Mat3 q_nu_nu = Q.block<3, 3>(3, 3);
    Mat3 q_nu_rho = Q.block<3, 3>(3, 6);
    Mat3 q_rho_rho = Q.bottomRightCorner<3, 3>();

    A_q.topLeftCorner<3, 3>() = brackets(q_phi_phi);
    A_q.block<3, 3>(3, 0) = brackets(q_phi_nu + q_phi_nu.transpose());
    A_q.block<3, 3>(3, 3) = brackets(q_phi_phi);
    A_q.bottomLeftCorner<3, 3>() = brackets(q_phi_rho + q_phi_rho.transpose());
    A_q.bottomRightCorner<3, 3>() = brackets(q_phi_phi);

    Mat9 B(Mat9::Zero());

    B.topLeftCorner<3, 3>() = brackets(sigma_phi_phi, q_phi_phi);
    B.block<3, 3>(3, 0) = brackets(sigma_phi_phi, q_phi_nu) + brackets(sigma_nu_phi, q_phi_phi);
    B.block<3, 3>(0, 3) = B.block<3, 3>(3, 0).transpose();
    B.block<3, 3>(6, 0) = brackets(sigma_phi_phi, q_phi_rho) + brackets(sigma_rho_phi, q_phi_phi);
    B.block<3, 3>(0, 6) = B.block<3, 3>(6, 0).transpose();
    B.block<3,3>(3,3) = brackets(sigma_phi_phi,q_nu_nu) + brackets(sigma_phi_nu,q_nu_phi) +
                        brackets(sigma_nu_phi, q_nu_phi) + brackets(sigma_nu_nu,q_phi_phi);

    B.block<3,3>(3,6) = brackets(sigma_nu_rho,q_phi_phi) + brackets(sigma_phi_rho,q_nu_phi) + 
                        brackets(sigma_nu_phi,q_rho_phi) + brackets(sigma_phi_phi, q_nu_rho);
    B.block<3,3>(6,3) = B.block<3,3>(3,6).transpose();
    B.bottomRightCorner<3, 3>() = brackets(sigma_phi_phi,q_rho_rho) + brackets(sigma_phi_rho,q_rho_phi) + 
                        brackets(sigma_rho_phi,q_phi_rho) + brackets(sigma_rho_rho,q_phi_phi);

    S_4th = 1./12.*(A_sigma*Q + Q*A_sigma.transpose() + A_q*sigma + sigma*A_q.transpose()) + 1./4.*B;

    return S_4th;
}
