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
    this->estimate_plane();//From EF center plane
    r1_.clear();r2_.clear();r3_.clear();points_.clear();
    Mat31 residual;
    for (auto &St: S_)
    {
        // residual 1:
        // n_k * l_1 ||normal'*R_k*v_1||^2
        //    where n_k is the number of points, l_1, max eigenvalue. and R_k the transformation
        residual = v3_.dot(v1_);
        points_.push_back(St(3,3));
        r1_.emplace_back(residual);

    }
}

void EigenFactorPlaneCoordinatesAlign::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    for (auto &St: S_)
    {
        Mat61 jacobian = Mat61::Zero();
        Mat6 hessian = Mat6::Zero();
        for (uint_t i = 0 ; i < 6; i++)
        {

        }
        J_.push_back(jacobian);
        H_.push_back(hessian);

    }
}

void EigenFactorPlaneCoordinatesAlign::evaluate_chi2()
{
    chi2_ = 0.0;
}

void EigenFactorPlaneCoordinatesAlign::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (auto &Qt: Q_)
    {
        accumulatedQ_ += Qt;
    }

    // Center the plane requires a transformation (translation) such that
    // pi_centered = [n, 0], such that T^{-\top} * pi_centered = pi,
    // This only hold for when T^{-\top} = [I, -n d].
    // and n d = - sum{p} / N = -E{x}    from the centered calculation of a plane
    Tcenter_.topRightCorner<3,1>() =  -accumulatedQ_.topRightCorner<3,1>()/accumulatedQ_(3,3);
    //std::cout << "T center = " << Tcenter_ <<  std::endl;

    //std::cout << "Q= \n" << accumulatedQ_ <<  std::endl;

    accumulatedCenterQ_ = Tcenter_ * accumulatedQ_ * Tcenter_.transpose();

    //std::cout << "new function Q= \n" << accumulatedCenterQ_ <<  std::endl;


    // Only needs Lower View from Q (https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html)
    Eigen::SelfAdjointEigenSolver<Mat3> es;
    es.computeDirect(accumulatedCenterQ_.topLeftCorner<3,3>());

    // other values, according to its formulation
    lambda_1_ = es.eigenvalues()(2);
    lambda_2_ = es.eigenvalues()(1);
    lambda_3_ = es.eigenvalues()(0);

    v1_ = es.eigenvectors().col(2);
    v2_ = es.eigenvectors().col(1);
    v3_ = es.eigenvectors().col(0);// this is the normal


}



