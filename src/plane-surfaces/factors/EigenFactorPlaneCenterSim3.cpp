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
 * EigenFactorPlanceCenter.cpp
 *
 *  Created on: Sept 1, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneCenterSim3.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/Sim3.hpp"
#include "mrob/utils_lie_differentiation.hpp"

using namespace mrob;

EigenFactorPlaneCenterSim3::EigenFactorPlaneCenterSim3(Factor::robustFactorType robust_type):
        EigenFactorPlaneBase(robust_type),
        planeEstimationUnit_{Mat41::Zero()},
        Tcenter_(Mat4::Identity())
{
}

void EigenFactorPlaneCenterSim3::evaluate_residuals()
{
    this->estimate_plane();
}

void EigenFactorPlaneCenterSim3::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    for (auto &Qt: Q_)
    {
        Mat71 jacobian = Mat71::Zero();
        Mat7 hessian = Mat7::Zero();
        // calculate gradient
        Mat<4,7> grad;
        grad = gradient_sim3_Q_x_pi(Qt,planeEstimation_);
        jacobian =  grad.transpose() * planeEstimation_;

        // calculate hessian ONLY Upper Trianlar view
        //tested: pi_t_x_hessian_Q_x_pi(),coincident with EFcenter-element-by-element implementation
        Mat7 pi_t_G_time_Q_grad;
        pi_t_G_time_Q_grad.triangularView<Eigen::Upper>() = 2.0*pi_t_times_lie_generatives_sim3(planeEstimation_)*grad;

        // Cross term dpi * dQ*pi, where dpi/dxi_i = Q^-1 dQ/dxi_i pi.
        // Not calculaterd since contribution is minimal

        // sum of all terms
        hessian.triangularView<Eigen::Upper>() =
                pi_t_x_hessian_sim3_Q_x_pi(Qt,planeEstimation_) +
                //grad_pi_time_Q_grad + // crosterm due to plane x dQ, excluded
                pi_t_G_time_Q_grad; //slihglty better than EFcenter
        J_.push_back(jacobian);
        H_.push_back(hessian);
    }
}

void EigenFactorPlaneCenterSim3::evaluate_chi2()
{
    // Point 2 plane exact error requires chi2 = pi' Q pi
    chi2_ = planeEstimation_.dot(accumulatedQ_ * planeEstimation_);

    //std::cout << ", error lambda = " << planeError_ << ", error projected plane = " << chi2_ << std::endl;
}


void EigenFactorPlaneCenterSim3::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();

    // Check for empty EF (no points)
    if (accumulatedQ_.sum()< 1e-4)
    {
        planeEstimation_.setZero();
        return;
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
    planeEstimationUnit_.head<3>() = es.eigenvectors().col(0);
    planeEstimationUnit_(3) = 0.0;

    planeEstimation_ = Sim3(Tcenter_).inv().transform_plane(planeEstimationUnit_);

    //std::cout << "\n and solution plane = \n" << planeEstimationUnit_ <<  std::endl;
    //std::cout << "plane estimation error (0): " << es.eigenvalues() <<  std::endl;

}




