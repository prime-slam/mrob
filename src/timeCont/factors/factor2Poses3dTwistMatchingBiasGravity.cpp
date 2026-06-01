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

#include <algorithm>
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
        Factor(9,27,robust_type), obs_(observation), W_(obsInf), delta_t_(0.0)
{
    // Since there are so manay nodes, let's impose order:
    // [0] -> Node Origin   size = 9
    // [1] -> Node Traget   size = 9
    // [2] -> Bias Acc      size = 3
    // [3] -> Bias gyro     size = 3
    // [4] -> gravity       size = 3
    std::vector<uint_t>  nodes_ids;
    nodes_ids.push_back(nodeOrigin->get_id());
    nodes_ids.push_back(nodeTarget->get_id());
    nodes_ids.push_back(nodeBiasAcc->get_id());
    nodes_ids.push_back(nodeBiasGyro->get_id());
    nodes_ids.push_back(nodeGravity->get_id());

    
    std::vector<uint_t> sizes_vector = {9,9,3,3,3};
    
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
    // [4] -> gravity       size = 3
    uint_t idx_origin = original_to_sorted_index_[0];
    uint_t idx_target = original_to_sorted_index_[1];
    uint_t idx_bias_acc = original_to_sorted_index_[2];
    uint_t idx_bias_gyro = original_to_sorted_index_[3];
    uint_t idx_gravity = original_to_sorted_index_[4];

    Mat5 state_origin; 
    state_origin = get_neighbour_nodes()->at(idx_origin)->get_state();
    SE3tc Tx_origin = SE3tc(state_origin);
    Mat5 state_target = get_neighbour_nodes()->at(idx_target)->get_state();
    SE3tc Tx_target_inv = SE3tc(state_target).inv();
    delta_t_ = get_neighbour_nodes()->at(idx_target)->get_time_stamp() - 
            get_neighbour_nodes()->at(idx_origin)->get_time_stamp();
    Mat31 bias_acc, bias_omega, gravity;
    bias_acc = get_neighbour_nodes()->at(idx_bias_acc)->get_state();
    bias_omega = get_neighbour_nodes()->at(idx_bias_gyro)->get_state();
    gravity = get_neighbour_nodes()->at(idx_gravity)->get_state();
    gravity = Tx_origin.R().transpose() * gravity; //transforming gravity in the gloabl frame, to the local frame at origin
    
    // constructor: omega, acc, dt
    SE3tc T_imu_integration = SE3tc( obs_.head(3) - bias_omega, obs_.tail(3) - bias_acc - gravity, delta_t_);

    T_target_inv_origin_ = Tx_target_inv * Tx_origin;
    SE3tc dT =  T_target_inv_origin_ * T_imu_integration;

    
    r_ = dT.vel().Ln();
}
void Factor2Poses3dTwistMatchingBiasGravity::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    // jacobian_node_index_[i] maps from original position i to jacobian column:
    
    Mat<10,10> inverse_jacobian;
    xi_target_inv_origin_ << T_target_inv_origin_.Ln();
    inverse_jacobian = inv_left_jacobian_tc(xi_target_inv_origin_);
    Mat9 clip_inverse_jacobian;
    clip_inverse_jacobian = inverse_jacobian.topLeftCorner<9,9>();
    //Origin (9x9 block)
    J_.block<9,9>(0,jacobian_node_index_[0]) =  clip_inverse_jacobian * T_target_inv_origin_.adj_vel();//TODO needs gravity
    //Target (9x9 block)
    J_.block<9,9>(0,jacobian_node_index_[1]) =  -clip_inverse_jacobian;
    //BiasAcc (9x3 block)
    J_.block<9,3>(0,jacobian_node_index_[2]) = -delta_t_ * J_.block<9,3>(0,jacobian_node_index_[0]);
    //BiasGyro (9x3 block)
    J_.block<9,3>(0,jacobian_node_index_[3]) = -delta_t_ * J_.block<9,3>(0,jacobian_node_index_[0] + 6);
    // Gravity (9x3 block)
    J_.block<9,3>(0,jacobian_node_index_[4]) = -delta_t_ * J_.block<9,3>(0,jacobian_node_index_[0] + 6);

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

