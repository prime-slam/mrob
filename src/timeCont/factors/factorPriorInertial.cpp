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
#include "mrob/factors/nodeGravity3d.hpp"
#include "mrob/SO3.hpp"

#include <iostream>

using namespace mrob;


FactorPriorInertial::FactorPriorInertial(const Mat5 &observationPoseTarget,
                    const Mat91 &obsBias,
                    std::shared_ptr<Node> &nodePoseTarget,
                    std::shared_ptr<Node> &nodeBiasGyro,
                    std::shared_ptr<Node> &nodeBiasAcc,
                    std::shared_ptr<Node> &nodeGravity,
                    const Mat<17,17> &obsInf, 
                    Factor::robustFactorType robust_type):
            Factor(17,17, robust_type), r_(Vect<17>::Zero()), W_(obsInf), J_(Mat<17,17>::Zero()),
            Tobs_(observationPoseTarget), xi_pose_(Mat101::Zero())
{

    obs_bias_gyro_ = obsBias.head(3);
    obs_bias_acc_= obsBias.segment<3>(3);
    obs_gravity_ = obsBias.tail(3);
    gravity_norm_ = obs_gravity_.norm();
    // Chart of the gravity residual: E = -hat(g_bar_unit) B(g_bar), so that dr/d(delta_g) = I at the mean
    E_ = -hat3(obs_gravity_ / gravity_norm_) * gravity_tangent_basis(obs_gravity_);

    // Ordering here MUST be the same. The Jacobian columns follow it.
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
    // Right-error residual r = Ln( Tobs^{-1} X ). With X = Tobs Exp(delta) this is r = delta to first order,
    // independent of the absolute time. The observation is re-stamped at the node time: the prior is a statement
    // about the state at that instant, so the time component of Tobs^{-1} X is exactly zero.
    const Mat5 x_target = get_neighbour_nodes()->at(0)->get_state();
    Mat5 T_obs_mat = Tobs_.T();
    T_obs_mat(4,3) = x_target(4,3);
    const SE3tc Tobs_inv = SE3tc(T_obs_mat).inv();
    const Mat5 rel = Tobs_inv.T() * x_target;
    xi_pose_ = SE3tc(rel).Ln();
    r_.head(9) << xi_pose_.head(9);

    // linear terms
    const Mat31 bias_gyro = get_neighbour_nodes()->at(1)->get_state();
    r_.segment<3>(9) << bias_gyro - obs_bias_gyro_;

    const Mat31 bias_acc = get_neighbour_nodes()->at(2)->get_state();
    r_.segment<3>(12) << bias_acc - obs_bias_acc_;

    // gravity on S^2: orthographic chart at the prior mean, in radians
    const Mat31 gravity = get_neighbour_nodes()->at(3)->get_state();
    r_.segment<2>(15) << E_.transpose() * (gravity - obs_gravity_) / gravity_norm_;
}

void FactorPriorInertial::evaluate_jacobians()
{
    J_.setZero();
    // Right perturbation of the pose: Ln(Exp(r) Exp(delta)) = r + Jr^{-1}(r) delta
    const Mat<10,10> Jr_inv = inv_right_jacobian_tc(xi_pose_);
    J_.topLeftCorner<9,9>() = Jr_inv.topLeftCorner<9,9>();
    J_.block<3,3>(9,9) = Mat3::Identity();
    J_.block<3,3>(12,12) = Mat3::Identity();
    // gravity: d g / d delta = -hat(g) B(g)
    const Mat31 gravity = get_neighbour_nodes()->at(3)->get_state();
    const Mat<3,2> dg = -hat3(gravity) * gravity_tangent_basis(gravity);
    J_.block<2,2>(15,15) = E_.transpose() * dg / gravity_norm_;
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
