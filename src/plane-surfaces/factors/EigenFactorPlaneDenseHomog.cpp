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
 * EigenFactorPlaneDenseHomog.cpp
 *
 *  Created on: Aug 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneDenseHomog.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"
#include "mrob/utils_lie_differentiation.hpp"

using namespace mrob;

EigenFactorPlaneDenseHomog::EigenFactorPlaneDenseHomog(Factor::robustFactorType robust_type):
        EigenFactorPlaneBase(robust_type)
{
}

void EigenFactorPlaneDenseHomog::evaluate_residuals()
{
    this->estimate_plane();
}

void EigenFactorPlaneDenseHomog::evaluate_jacobians()
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
        grad = gradient_Q_x_pi(Qt,planeEstimation_);
        gradQ_xi_times_pi_.push_back(grad);
        jacobian =  grad.transpose() * planeEstimation_;

        // calculate hessian ONLY Upper Trianlar view
        //tested: pi_t_x_hessian_Q_x_pi(),coincident with EFcenter-element-by-element implementation
        Mat6 pi_t_G_time_Q_grad;
        pi_t_G_time_Q_grad.triangularView<Eigen::Upper>() = 2.0*pi_t_times_lie_generatives(planeEstimation_)*grad;

        // Cross term dpi * dQ*pi, where dpi/dxi_i = Q^-1 dQ/dxi_i pi. Due to symetry, both temrs are equal and sum (-> 2.0*)
        Mat6 grad_pi_time_Q_grad;
        grad_pi_time_Q_grad.triangularView<Eigen::Upper>() = 2.0*grad.transpose()*Q_inv_no_kernel_*grad;

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

void EigenFactorPlaneDenseHomog::evaluate_chi2()
{
    // Point 2 plane exact error requires chi2 = pi' Q pi
    chi2_ = planeEstimation_.dot( accumulatedQ_ * planeEstimation_ );
    //std::cout << "plane = " << planeEstimation_ << "\n Q = \n" << accumulatedQ_ << std::endl;
}

void EigenFactorPlaneDenseHomog::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();

    // Check for empty EF (no points)
    if (accumulatedQ_.sum()< 1e-4)
    {
        planeEstimation_.setZero();
        Q_inv_no_kernel_.setZero();
        return;
    }


    // Center the plane requires a transformation (translation) such that
    // pi_centered = [n, 0], such that T^{-\top} * pi_centered = pi,
    // This only hold for when T^{-\top} = [I, -n d].
    // and n d = - sum{p} / N = -E{x}    from the centered calculation of a plane
    Mat4 Tcenter = Mat4::Identity();
    Tcenter.topRightCorner<3,1>() =  -accumulatedQ_.topRightCorner<3,1>()/accumulatedQ_(3,3);
    //std::cout << "T center = " << Tcenter <<  std::endl;


    Mat4 accumulatedCenterQ;
    accumulatedCenterQ = Tcenter * accumulatedQ_ * Tcenter.transpose();


    // Only needs Lower View from Q (https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html)
    Eigen::SelfAdjointEigenSolver<Mat3> es;
    es.computeDirect(accumulatedCenterQ.topLeftCorner<3,3>());
    // This plane center is more stable when calcualating a solution than its counterpart in 4d
    // Most likely it is how the error leaks from the distance componet and breaks the normal component,
    // that needs to be projected (regenerated) on the 4x4 case. this way, we make sure it is a unit vector by construction.
    // Right after the calcuation, we convert back to the correct reference frame.
    Mat41 planeEstimationCenter;
    planeEstimationCenter.head<3>() = es.eigenvectors().col(0);
    planeEstimationCenter(3) = 0.0;


    planeEstimation_ = Tcenter.transpose() * planeEstimationCenter;

    // Calcualte almost inverse of Q for later derivatives:
    // option 1, full inverse: slightly inacurate solution.
    //Q_inv_no_kernel_ = accumulatedQ_.inverse();

    // Option 2. Direct eig decomposition and inverse
    Eigen::SelfAdjointEigenSolver<Mat4> es4;
    es4.compute(accumulatedQ_);
    matData_t lambda_plane = es4.eigenvalues()(0);
    Mat4 other_eigenvectors, other_eigenvectors_multiplied;
    other_eigenvectors = es4.eigenvectors();
    other_eigenvectors.col(0) *= 0.0;
    //std::cout << "other eignevect \n" << other_eigenvectors <<std::endl;
    other_eigenvectors_multiplied.col(0) = 0.0 * other_eigenvectors.col(0);
    other_eigenvectors_multiplied.col(1) = 1.0/(lambda_plane - es4.eigenvalues()(1)) * other_eigenvectors.col(1);
    other_eigenvectors_multiplied.col(2) = 1.0/(lambda_plane - es4.eigenvalues()(2)) * other_eigenvectors.col(2);
    other_eigenvectors_multiplied.col(3) = 1.0/(lambda_plane - es4.eigenvalues()(3)) * other_eigenvectors.col(3);
    //std::cout << "other eignevect mult \n" << other_eigenvectors_multiplied <<std::endl;
    //std::cout << "Q_inv_ 4x4 inverse =\n" << other_eigenvectors * other_eigenvectors_multiplied.transpose() <<std::endl;
    Q_inv_no_kernel_ = other_eigenvectors * other_eigenvectors_multiplied.transpose();
    //std::cout << "Difference  =\n" << (other_eigenvectors * other_eigenvectors_multiplied.transpose()-Q_inv_no_kernel_).squaredNorm() <<std::endl;



}

bool EigenFactorPlaneDenseHomog::get_hessian(MatRef H, mrob::factor_id_t id_i, mrob::factor_id_t id_j) const
{
    // this condition should always hold since ids are taken from neubouring nodes, but in case useage changes.
    if (reverseNodeIds_.count(id_i) == 0   ||  reverseNodeIds_.count(id_j) == 0)
        return false;

    uint_t localId1 = reverseNodeIds_.at(id_i);
    uint_t localId2 = reverseNodeIds_.at(id_j);
    if(id_i == id_j)
    {
        H = H_.at(localId1);
        return true;
    }
    else
    {
        // cross terms as
        H = 2.0*gradQ_xi_times_pi_.at(localId1).transpose() * Q_inv_no_kernel_* gradQ_xi_times_pi_.at(localId2);
        return true;
    }
}
