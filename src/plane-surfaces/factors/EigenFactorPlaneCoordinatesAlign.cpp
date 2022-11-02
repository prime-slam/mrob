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
 * EigenFactorPlaneCoordinatesAlign.cpp
 *
 *  Created on: Oct 31, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneCoordinatesAlign.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"

using namespace mrob;

EigenFactorPlaneCoordinatesAlign::EigenFactorPlaneCoordinatesAlign(Factor::robustFactorType robust_type):
        EigenFactorPlaneCenter(robust_type)
{
}

void EigenFactorPlaneCoordinatesAlign::evaluate_residuals()
{
    this->EigenFactorPlaneCenter::estimate_plane();

    // calculate V's and lambda's for each pose St
    this->estimate_planes_at_poses();

    r1_.clear();r2_.clear();r3_.clear();
    uint_t nodeIdLocal = 0;
    Mat31 normal = this->get_estimate_normal();// these are in global coordinates
    Mat31 mean = this->get_estimate_mean();
    for (auto &St: S_)
    {
        // residual 1:
        // n_k * l_1 ||normal'*R_k*v_1||^2
        //    where n_k is the number of points, l_1, max eigenvalue. and R_k the transformation
        Mat4 pose_state = this->neighbourNodes_[nodeIdLocal]->get_state();
        SE3 T(pose_state);
        r1_.push_back(normal.dot(T.R() * v1_[nodeIdLocal]));

        // residual 2, same for vor VAP 2
        r2_.push_back(normal.dot(T.R() * v2_[nodeIdLocal]));

        // residual 3
        Mat31 local_mean = St.topRightCorner<3,1>()/St(3,3);
        r3_.push_back(normal.dot(T.transform(local_mean) - mean));

        // residual 3:
        nodeIdLocal++;
    }
}

void EigenFactorPlaneCoordinatesAlign::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    uint_t nodeIdLocal = 0;
    Mat31 normal = this->get_estimate_normal();// these are in global coordinates
    for (auto &St: S_)
    {
        Mat61 jacobian = Mat61::Zero(), dr;
        Mat6 hessian = Mat6::Zero();
        Mat<3,6> diff = Mat<3,6>::Zero();
        Mat4 pose_state = this->neighbourNodes_[nodeIdLocal]->get_state();
        SE3 T(pose_state);

        // Jacobian 1 = r * W * dr/dxi = r1 * N*lambda1 * normal'[-(R v1)^ | 000 ]
        // ------------------------------------------------------------------------
        matData_t k = n_points_[nodeIdLocal]*lambda_1_[nodeIdLocal];
        diff.topLeftCorner<3,3>() = -hat3(T.R()*v1_[nodeIdLocal]);
        dr = normal.transpose() * diff;
        jacobian = k*r1_[nodeIdLocal]*dr;

        // Hessian 1 = W * dr' * dr
        hessian = k* dr * dr.transpose();


        // Jacobian 2 = r * W * dr/dxi = r2 * N*lambda2 * normal'[-(R v2)^ | 000 ]
        // ------------------------------------------------------------------------
        k = n_points_[nodeIdLocal]*lambda_2_[nodeIdLocal];
        diff.Zero();
        diff.topLeftCorner<3,3>() = -hat3(T.R()*v2_[nodeIdLocal]);
        dr = normal.transpose()  *diff;
        jacobian += k*r2_[nodeIdLocal]*dr;
        hessian += k* dr * dr.transpose();


        // Jacobian 3 = r * W * dr/dxi = r3 * N * normal'[-(T mu_t)^ | I ]
        k = n_points_[nodeIdLocal];
        diff.Zero();
        Mat31 local_mean = St.topRightCorner<3,1>()/St(3,3);
        diff.topLeftCorner<3,3>() = -hat3(T.transform(local_mean));
        diff.topRightCorner<3,3>() = Mat3::Identity();
        dr = normal.transpose() * diff;
        jacobian += k*r3_[nodeIdLocal]*dr;
        hessian += k* dr * dr.transpose();

        J_.push_back(jacobian);
        H_.push_back(hessian);
        nodeIdLocal++;
    }
}

void EigenFactorPlaneCoordinatesAlign::evaluate_chi2()
{
    chi2_ = 0.0;
    uint_t nodeIdLocal = 0;
    for (auto r1 : r1_)
    {
        chi2_ += r1 * r1 * n_points_[nodeIdLocal] * lambda_1_[nodeIdLocal];
        chi2_ += r2_[nodeIdLocal] * r2_[nodeIdLocal]* n_points_[nodeIdLocal] * lambda_2_[nodeIdLocal];
        chi2_ += r3_[nodeIdLocal] * r3_[nodeIdLocal]* n_points_[nodeIdLocal];
        nodeIdLocal++;
    }
    chi2_ *= 0.5;
}

Mat31 EigenFactorPlaneCoordinatesAlign::get_estimate_normal() const
{
    return planeEstimationUnit_.head<3>();
}

Mat31 EigenFactorPlaneCoordinatesAlign::get_estimate_mean() const
{
    return accumulatedQ_.topRightCorner<3,1>()/accumulatedQ_(3,3);
}

void EigenFactorPlaneCoordinatesAlign::estimate_planes_at_poses()
{
    if(lambda_1_.empty())
    {
        Mat4 Tcenter = Mat4::Identity();
        Mat4 cov_t  = Mat4::Identity();
        Eigen::SelfAdjointEigenSolver<Mat3> es;
        for (auto &St : S_)
        {
            Tcenter.topRightCorner<3,1>() =  -St.topRightCorner<3,1>()/St(3,3);
            cov_t =  Tcenter * St * Tcenter.transpose();
            es.computeDirect(cov_t.topLeftCorner<3,3>());
            lambda_1_.emplace_back(es.eigenvalues()(2));
            v1_.emplace_back(es.eigenvectors().col(2));
            lambda_2_.emplace_back(es.eigenvalues()(1));
            v2_.emplace_back(es.eigenvectors().col(1));
            n_points_.emplace_back(St(3,3));
        }

    }
}
