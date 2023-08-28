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
 * EigenFactorPlaneDense.cpp
 *
 *  Created on: Aug 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneDense.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"
#include "mrob/utils_lie_differentiation.hpp"

using namespace mrob;

EigenFactorPlaneDense::EigenFactorPlaneDense(Factor::robustFactorType robust_type):
        EigenFactorPlaneBase(robust_type)
{
}

void EigenFactorPlaneDense::evaluate_residuals()
{
    this->estimate_plane();
}

void EigenFactorPlaneDense::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    gradQ_xi_times_pi_.clear();
    for (auto &Qt: Q_)
    {
        Mat61 jacobian = Mat61::Zero();
        Mat6 hessian = Mat6::Zero();

        // calculate gradient
        Mat<4,6> grad;
        gradQ_xi_times_pi_.push_back(grad);
        grad = gradient_Q_x_pi(Qt,planeEstimation_);
        jacobian =  grad.transpose() * planeEstimation_;

        // calculate hessian ONLY Upper Trianlar view
        //tested: pi_t_x_hessian_Q_x_pi(),coincident with EFcenter-element-by-element implementation
        Mat6 pi_t_G_time_Q_grad;
        pi_t_G_time_Q_grad.triangularView<Eigen::Upper>() = 2.0*pi_t_times_lie_generatives(planeEstimation_)*grad;

        // Cross term dpi * dQ*pi, where dpi/dxi_i = Q^-1 dQ/dxi_i pi.
        Mat6 grad_pi_time_Q_grad;
        grad_pi_time_Q_grad.triangularView<Eigen::Upper>() = grad.transpose()*Q_inv_no_kernel_*grad;

        // sum of all terms
        hessian.triangularView<Eigen::Upper>() =
                pi_t_x_hessian_Q_x_pi(Qt,planeEstimation_) +
                grad_pi_time_Q_grad + // crosterm due to plane x dQ
                pi_t_G_time_Q_grad; //slihglty better than EFcenter


        J_.push_back(jacobian);
        H_.push_back(hessian);
        // need to store 
        //std::cout << "Jacobian =\n" << hessian <<std::endl;
    }
}

void EigenFactorPlaneDense::evaluate_chi2()
{
    // Point 2 plane exact error requires chi2 = pi' Q pi
    chi2_ = planeEstimation_.dot( accumulatedQ_ * planeEstimation_ );
    //chi2_ = planeError_; // this is the scaled error
}


MatRefConst EigenFactorPlaneDense::get_jacobian(mrob::factor_id_t id) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlaneDense::get_jacobian: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return J_.at(localId);
}

MatRefConst EigenFactorPlaneDense::get_hessian(mrob::factor_id_t id) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlaneDense::get_hessian: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return H_.at(localId);
}


MatRefConst EigenFactorPlaneDense::get_hessian_block(mrob::factor_id_t id1, mrob::factor_id_t id2) const
{
    assert(reverseNodeIds_.count(id1)   && "EigenFactorPlaneDense::get_hessian: element id1 not found");
    assert(reverseNodeIds_.count(id2)   && "EigenFactorPlaneDense::get_hessian: element id2 not found");

    Mat6 block_hessian;
    if(id1 == id2)
    {
        uint_t localId = reverseNodeIds_.at(id1);
        block_hessian = H_.at(localId);
    }
    else
    {
        // fetch cross terms and return the value
    }
    return block_hessian;
}