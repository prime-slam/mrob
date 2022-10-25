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
 * EigenFactorPoint.cpp
 *
 *  Created on: Oct 24, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPoint.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>


using namespace mrob;

EigenFactorPoint::EigenFactorPoint(Factor::robustFactorType robust_type):
        EigenFactorPlane(robust_type)
{
}

void EigenFactorPoint::evaluate_residuals()
{
    // 1) calculate the Q matrices with new transformations
    this->calculate_all_matrices_S();//only if not calculate before
    this->calculate_all_matrices_Q();

    // 2) Calculate the residual, same structure as in EigenFactorPlane::calculate_matrices_Q()
    //    r = To^{-1}T * mu(Qi) - mu(Q0)
    r_.clear();
    transformed_mu_.clear();
    uint_t nodeIdLocal = 0;
    Mat4 initial_transform = this->neighbourNodes_[nodeIdLocal]->get_state();
    T_ini_inv_ = SE3(initial_transform).inv();
    Mat31 initial_mean_point;
    initial_mean_point = S_[0].topRightCorner<3,1>()/S_[0](3,3);// TODO we should do a method for this
    for (auto &S : S_)
    {
        // 3) get current transformation. Here we follow the same scheme as in Factor1PosePoint2Point
        Mat4 Tnode = this->neighbourNodes_[nodeIdLocal]->get_state();
        SE3 T(Tnode);
        Mat31 local_mean_point = S.topRightCorner<3,1>()/S(3,3);
        Mat31 Tmu = T.transform(local_mean_point);
        transformed_mu_.emplace_back(Tmu);
        // we need to store T_t * mu, and then transform the full sequence T0^{-1}*T_t * mu
        Mat31 residual = T_ini_inv_.transform(Tmu) - initial_mean_point;
        r_.emplace_back(residual);
        nodeIdLocal++;
    }
}

void EigenFactorPoint::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    uint_t nodeIdLocal = 0;
    for (auto &rt: r_)
    {
        // Jacobian = dr/dxi_t' *W_t *  r_t
        matData_t W =  S_[nodeIdLocal](3,3); // Information is simply the number of points, weighted solution
        Mat31 Tx_t = transformed_mu_.at(nodeIdLocal);
        Mat61 jacobian = Mat61::Zero();
        Mat<3,6> dr;
        dr << -hat3(Tx_t) , Mat3::Identity();
        dr = T_ini_inv_.R() * dr;
        jacobian = W * dr.transpose() * rt;
        J_.push_back(jacobian);
        // Hessian = dr/dxi_t' * W_t * dr/dxi_t
        Mat6 hessian = Mat6::Zero();
        hessian = W* dr.transpose() * dr;
        H_.push_back(hessian);
        nodeIdLocal++;
    }
}

void EigenFactorPoint::evaluate_chi2()
{
    chi2_ = 0.0;
    uint_t nodeIdLocal = 0;
    for (auto &rt: r_)
    {
        chi2_ += S_[nodeIdLocal](3,3) * rt.dot(rt);
        nodeIdLocal++;
    }
    chi2_ *= 0.5;
}


