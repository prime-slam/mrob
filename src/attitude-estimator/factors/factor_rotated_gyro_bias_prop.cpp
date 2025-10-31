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
* FactorRotatedGyroBiasProp.hpp
*
*  Created on: Oct 25, 2025
*      Author: Ivan Kakurin
*              i.kakurin@skoltech.ru
*              Gonzalo Ferrer
*              g.ferrer@skoltech.ru
*              Mobile Robotics Lab, Skoltech 
*/


#include "mrob/factors/factor_rotated_gyro_bias_prop.hpp"

#include <iostream>

using namespace mrob;

FactorRotatedGyroBiasProp::FactorRotatedGyroBiasProp(const Mat31 &gyroscope, const double &dt,
            std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget,
            std::shared_ptr<Node> &nodeBias,
            std::shared_ptr<Node> &nodeRotation,
            const Mat3 &obsInf, 
            Factor::robustFactorType robust_type):
        Factor(3,12, robust_type),
        dt_(dt), gyro_(gyroscope),W_(obsInf),
        J_(Mat<3,12>::Zero())
{
    bias_ = nodeBias->get_state();
    Robs_.exp(hat3((gyro_ - bias_) * dt_));

    // [0] -> Node Origin   size = 3
    // [1] -> Node Traget   size = 3
    // [2] -> Bias gyro     size = 3
    // [3] -> Rotation     size = 3
    std::vector<uint_t>  nodes_ids;
    nodes_ids.push_back(nodeOrigin->get_id());
    nodes_ids.push_back(nodeTarget->get_id());
    nodes_ids.push_back(nodeBias->get_id());
    nodes_ids.push_back(nodeRotation->get_id());

    
    std::vector<uint_t> sizes_vector = {3,3,3,3};
    
    //function from factors.hpp, for factors that have several nodes connected tha require to maintain order, mostly for the Jacobian
    mrob::getJacobianIndexUnorderedNodes(nodes_ids, sizes_vector, original_to_ordered_index_, jacobian_node_index_);


    
    std::vector<std::shared_ptr<Node> >  smart_pointers_vector = {nodeOrigin, nodeTarget, nodeBias, nodeRotation};
    node_pos_in_ordered_list_ = std::vector<uint_t>(nodes_ids.size());
    uint_t i_node_in_ordered_list = 0;
    for (auto i : original_to_ordered_index_)
    {
        neighbourNodes_.push_back(smart_pointers_vector[i]);
        node_pos_in_ordered_list_[i] = i_node_in_ordered_list;
        i_node_in_ordered_list++;
    }

}



void FactorRotatedGyroBiasProp::evaluate_residuals()
{
    // From Origin we observe Target such that: R_o * R_obs = R_t
    // Rr = Rxo * Exp((w-b)*dt) * Rxt^-1

    Mat3 RxOrigin = get_neighbour_nodes()->at(node_pos_in_ordered_list_[0])->get_state();
    Mat3 RxTarget = get_neighbour_nodes()->at(node_pos_in_ordered_list_[1])->get_state();
    bias_ = get_neighbour_nodes()->at(node_pos_in_ordered_list_[2])->get_state();
    Mat3 RxRotation = get_neighbour_nodes()->at(node_pos_in_ordered_list_[3])->get_state();
    R_reference_ = SO3(RxRotation);
    Robs_.exp(hat3((gyro_ - bias_) * dt_));
    Rr_ = SO3(RxOrigin) * R_reference_ * Robs_ * R_reference_.inv() *SO3(RxTarget).inv();
    r_ = Rr_.ln_vee();
}

void FactorRotatedGyroBiasProp::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SO3 and small perturbations)
    Mat3 inv_left_jacobian, left_jacobian_bias;
    inv_left_jacobian = inv_left_jacobian_SO3(r_);
    J_.block<3,3>(0,jacobian_node_index_[0]) = inv_left_jacobian;
    J_.block<3,3>(0,jacobian_node_index_[1]) = -inv_left_jacobian*Rr_.adj();
    
    // dr/db
    bias_ = get_neighbour_nodes()->at(node_pos_in_ordered_list_[2])->get_state();
    left_jacobian_bias = left_jacobian_SO3((gyro_ - bias_) * dt_);
    Mat3 RxOrigin = get_neighbour_nodes()->at(node_pos_in_ordered_list_[0])->get_state();
    Mat3 adjoint_rt_rot;
    adjoint_rt_rot = RxOrigin * R_reference_.R();
    J_.block<3,3>(0,jacobian_node_index_[2]) = -dt_*inv_left_jacobian*adjoint_rt_rot*left_jacobian_bias;
    
    // dr/dR_rot
    Mat3 DR = RxOrigin * R_reference_.R() * Robs_.R() * R_reference_.inv().R();
    left_jacobian_bias = left_jacobian_SO3(SO3(DR).ln_vee());
    J_.block<3,3>(0,jacobian_node_index_[3]) = inv_left_jacobian * ( left_jacobian_bias *RxOrigin - DR);
    //J_.block<3,3>(0,jacobian_node_index_[3]) = inv_left_jacobian * RxOrigin * (Mat3::Identity() - R_reference_.R() * Robs_.R() * R_reference_.inv().R());
}


void FactorRotatedGyroBiasProp::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void FactorRotatedGyroBiasProp::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << gyro_
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Nodes " << neighbourNodes_.size()
            << std::endl;
}
