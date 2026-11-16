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
 * factor2Poses3dTwistMatchingBiasGravity.cpp
 *
 *  Created on: September 12, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3dTwistMatchingBiasGravity.hpp"
#include "mrob/factors/nodeGravity3d.hpp"
#include "mrob/SO3.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace mrob;

Factor2Poses3dTwistMatchingBiasGravity::Factor2Poses3dTwistMatchingBiasGravity(
        const Mat61 &observation, 
        std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, 
        std::shared_ptr<Node> &nodeBiasAcc, 
        std::shared_ptr<Node> &nodeBiasGyro,
        std::shared_ptr<Node> &nodeGravity,
        const Mat9 &obsInf, 
        Factor::robustFactorType robust_type):
        Factor(9,26,robust_type), obs_(observation), W_(obsInf), delta_t_(0.0)
{
    // Since there are so many nodes, let's impose order:
    // [0] -> Node Origin   size = 9
    // [1] -> Node Traget   size = 9
    // [2] -> Bias Acc      size = 3
    // [3] -> Bias gyro     size = 3
    // [4] -> gravity       size = 2 (S^2)
    std::vector<uint_t>  nodes_ids;
    nodes_ids.push_back(nodeOrigin->get_id());
    nodes_ids.push_back(nodeTarget->get_id());
    nodes_ids.push_back(nodeBiasAcc->get_id());
    nodes_ids.push_back(nodeBiasGyro->get_id());
    nodes_ids.push_back(nodeGravity->get_id());
    
    std::vector<uint_t> sizes_vector = {9,9,3,3,2};
    
    //function from factors.hpp, for factors that have so many nodes connected tha require to maintain order, mostly for the Jacobian
    std::vector<uint_t> original_to_ordered_index;
    mrob::getJacobianIndexUnorderedNodes(nodes_ids, sizes_vector, original_to_ordered_index, jacobian_node_index_);

    original_to_sorted_index_.resize(5);
    for (uint_t sorted_pos = 0; sorted_pos < original_to_ordered_index.size(); ++sorted_pos)
    {
        uint_t original_pos = original_to_ordered_index[sorted_pos];
        original_to_sorted_index_[original_pos] = sorted_pos;
    }
    std::vector<std::shared_ptr<Node> >  smart_pointers_vector = {nodeOrigin, nodeTarget, nodeBiasAcc, nodeBiasGyro, nodeGravity};
    for (auto i : original_to_ordered_index)
    {
        neighbourNodes_.push_back(smart_pointers_vector[i]);
    }
}



void Factor2Poses3dTwistMatchingBiasGravity::evaluate_residuals()
{
    // Nodes are stored in sorted order by ID in neighbourNodes_
    // Use original_to_sorted_index_ to map from original position to sorted position:
    // [0] -> Node Origin   size = 9
    // [1] -> Node Target   size = 9
    // [2] -> Bias Acc      size = 3
    // [3] -> Bias gyro     size = 3
    // [4] -> gravity       size = 2 (S^2)
    const uint_t idx_origin = original_to_sorted_index_[0];
    const uint_t idx_target = original_to_sorted_index_[1];
    const uint_t idx_bias_acc = original_to_sorted_index_[2];
    const uint_t idx_bias_gyro = original_to_sorted_index_[3];
    const uint_t idx_gravity = original_to_sorted_index_[4];

    const Mat5 state_origin = get_neighbour_nodes()->at(idx_origin)->get_state();
    const Mat5 state_target = get_neighbour_nodes()->at(idx_target)->get_state();
    delta_t_ = get_neighbour_nodes()->at(idx_target)->get_time_stamp() - 
            get_neighbour_nodes()->at(idx_origin)->get_time_stamp();
    const Mat31 bias_acc = get_neighbour_nodes()->at(idx_bias_acc)->get_state();
    const Mat31 bias_omega = get_neighbour_nodes()->at(idx_bias_gyro)->get_state();
    gravity_ = get_neighbour_nodes()->at(idx_gravity)->get_state();

    const SE3tc Tx_origin(state_origin);
    R_origin_T_ = Tx_origin.R().transpose();
    gravity_local_ = R_origin_T_ * gravity_; // gravity in the start frame

    // Gravity as a separate element on the left of U (time 0): p = 1/2 g_l dt^2, v = -g_l dt.
    // The 1/2 g_l dt^2 compensates the cross term v_W * t_U of the group product, so that the total
    // contribution to the position is -1/2 g_l dt^2 (and -g_l dt to the velocity), constant in the start frame.
    Mat5 W_g = Mat5::Identity();
    W_g.block<3,1>(0,3) = 0.5 * gravity_local_ * delta_t_ * delta_t_;
    W_g.block<3,1>(0,4) = -gravity_local_ * delta_t_;

    // U = Exp(xi_u), xi_u = ( (w - bg) dt, 0, (a - ba) dt, dt )
    xi_imu_integration_.setZero();
    xi_imu_integration_.head<3>() = (obs_.head(3) - bias_omega) * delta_t_;
    xi_imu_integration_.segment<3>(6) = (obs_.tail(3) - bias_acc) * delta_t_;
    xi_imu_integration_(9) = delta_t_;
    T_imu_integration_ = SE3tc(xi_imu_integration_);

    T_target_inv_origin_ = SE3tc(state_target).inv() * Tx_origin;
    const Mat5 dT_mat = T_target_inv_origin_.T() * W_g * T_imu_integration_.T();
    xi_dT_ = SE3tc(dT_mat).Ln();
    // Residual in the SE3tc tangent (pose+vel); the time component is zero by construction
    r_ = xi_dT_.head(9);
}

void Factor2Poses3dTwistMatchingBiasGravity::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    // jacobian_node_index_[i] maps from original position i to jacobian column.
    //
    // Notation: r = Ln(dT), dT = G W U, G = T2^{-1} T1, right perturbations T_i -> T_i Exp(delta_i) (time of update 0).
    //   Jl^{-1}, Jr^{-1}: inverse left/right Jacobians evaluated at the RESIDUAL r (time 0).
    //   K = Jr^{-1}(r) Ad_{U^{-1}}.
    //
    //   T1:      dT' = G Exp(delta_1) W U  ->  J1 = Jl^{-1}(r) Ad_G  +  K E_phi   (E_phi: gravity expressed in the rotating start frame)
    //   T2:      J2 = -Jl^{-1}(r)
    //   ba, bg:  U(xi_u + d) = U Exp(Jr(xi_u) d)  -> J = Jr^{-1}(r) Jr(xi_u) S    (S: d xi_u / d b)
    //   g:       J = K E_g  (3-vector)  followed by the chain rule of the S^2 retraction  d g / d delta = -hat(g) B(g)
    J_.setZero();
    const Mat<10,10> Jl_inv_tc = inv_left_jacobian_tc(xi_dT_);
    const Mat<10,10> Jr_inv_tc = inv_right_jacobian_tc(xi_dT_);
    const Mat<10,10> Ad_G = T_target_inv_origin_.adj();
    const Mat<10,10> K = Jr_inv_tc * T_imu_integration_.inv().adj();

    // Origin (9x9)
    Mat<9,9> J1 = (Jl_inv_tc * Ad_G).topLeftCorner<9,9>();
    // The start-frame gravity g_l = R1^T g depends on the origin rotation: with R1 -> R1 Exp(delta_phi),
    // delta g_l = -hat(delta_phi) g_l  and  W Exp(e_W) = W(g_l + delta g_l)  with e_W = (0, 1/2 dt^2 delta g_l, -dt delta g_l, 0).
    Mat<10,3> E_phi = Mat<10,3>::Zero();
    E_phi.block<3,3>(3,0) = 0.5 * delta_t_ * delta_t_ * hat3(gravity_local_);
    E_phi.block<3,3>(6,0) = -delta_t_ * hat3(gravity_local_);
    J1.leftCols<3>() += (K * E_phi).topRows<9>();
    J_.block<9,9>(0, jacobian_node_index_[0]) = J1;

    // Target (9x9)
    J_.block<9,9>(0, jacobian_node_index_[1]) = -Jl_inv_tc.topLeftCorner<9,9>();

    // Bias acc and gyro through xi_u
    const Mat<10,10> Jr_u = right_jacobian_tc(xi_imu_integration_);
    const Mat<10,10> M_u = Jr_inv_tc * Jr_u;
    Mat<10,3> S_ba = Mat<10,3>::Zero();
    S_ba.block<3,3>(6,0) = -delta_t_ * Mat3::Identity();   // d xi_u(nu) / d ba
    J_.block<9,3>(0, jacobian_node_index_[2]) = (M_u * S_ba).topRows<9>();
    Mat<10,3> S_bg = Mat<10,3>::Zero();
    S_bg.block<3,3>(0,0) = -delta_t_ * Mat3::Identity();   // d xi_u(phi) / d bg
    J_.block<9,3>(0, jacobian_node_index_[3]) = (M_u * S_bg).topRows<9>();

    // Gravity: W depends on g_l = R1^T g
    Mat<10,3> E_g = Mat<10,3>::Zero();
    E_g.block<3,3>(3,0) = 0.5 * delta_t_ * delta_t_ * R_origin_T_;
    E_g.block<3,3>(6,0) = -delta_t_ * R_origin_T_;
    const Mat<9,3> J_g3 = (K * E_g).topRows<9>();
    const Mat<3,2> dg_ddelta = -hat3(gravity_) * gravity_tangent_basis(gravity_);   // S^2 chart of NodeGravity3d
    J_.block<9,2>(0, jacobian_node_index_[4]) = J_g3 * dg_ddelta;
}

void Factor2Poses3dTwistMatchingBiasGravity::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3dTwistMatchingBiasGravity::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
            << ", " << neighbourNodes_[1]->get_id()
            << std::endl;
}
