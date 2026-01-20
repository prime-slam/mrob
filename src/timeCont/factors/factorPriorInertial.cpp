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
 * FactorPriorInertial.cpp
 *
 *  Created on: Jan 20, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factorPriorInertial.hpp"

#include <iostream>

using namespace mrob;


FactorPriorInertial::FactorPriorInertial(const Mat5 &observationPoseTarget,
                    const Mat91 &obsBias,
                    std::shared_ptr<Node> &nodePoseTarget,
                    std::shared_ptr<Node> &nodeBiasGyro,
                    std::shared_ptr<Node> &nodeBiasAcc,
                    std::shared_ptr<Node> &nodeGravity,
                    const Mat<18,18> &obsInf, 
                    Factor::robustFactorType robust_type):
             Factor(18,18, robust_type), W_(obsInf), r_(Vect<18>::Zero()), J_(Mat<18,18>::Zero())
{

    obs_bias_gyro_ = obsBias.head(3);
    obs_bias_acc_= obsBias.segment<3>(3);
    obs_gravity_ = obsBias.tail(3);
    Tobs_target_inv_ = SE3tc(observationPoseTarget).inv();
    // Ordering here MUST be the same. TODO handle any order.
    // [0] Pose Target
    // [1] bias gyro
    // [2] bias acc
    // [3] gravity in the global frame
    neighbourNodes_.push_back(nodePoseTarget);
    neighbourNodes_.push_back(nodeBiasGyro);
    neighbourNodes_.push_back(nodeBiasAcc);
    neighbourNodes_.push_back(nodeGravity);
}

void FactorPriorInertial::evaluate_residuals()
{
    // Anchor residuals as r = x - obs
    // r = ln(X * Tobs^-1) = - ln(Tobs * x^-1)
    // NOTE Tobs is a global observation (reference identity)
    Mat5 x_target = get_neighbour_nodes()->at(0).get()->get_state();
    Tr_ = SE3tc(x_target) * Tobs_target_inv_;
    r_.head(9) << Tr_.Ln().head(9);

    // manifold for linear terms
    Mat31 bias_gyro = get_neighbour_nodes()->at(1)->get_state();
    r_.segment<3>(9) << bias_gyro - obs_bias_gyro_;

    Mat31 bias_acc = get_neighbour_nodes()->at(2)->get_state();
    r_.segment<3>(9+3) << bias_gyro - obs_bias_acc_;

    Mat31 bias_grav = get_neighbour_nodes()->at(3)->get_state();
    r_.segment<3>(9+6) << bias_grav - obs_gravity_;
}

void FactorPriorInertial::evaluate_jacobians()
{
    J_ = Mat<18,18>::Identity();
}

void FactorPriorInertial::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void FactorPriorInertial::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n"
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}
