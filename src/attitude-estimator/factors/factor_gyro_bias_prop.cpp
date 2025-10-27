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
* FactorGyroBiasProp.hpp
*
*  Created on: Oct 25, 2025
*      Author: Ivan Kakurin
*              i.kakurin@skoltech.ru
*              Gonzalo Ferrer
*              g.ferrer@skoltech.ru
*              Mobile Robotics Lab, Skoltech 
*/


#include "mrob/factors/factor_gyro_bias_prop.hpp"

#include <iostream>

using namespace mrob;

FactorGyroBiasProp::FactorGyroBiasProp(const Mat31 &gyroscope, const double &dt,
            std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget,
            std::shared_ptr<Node> &nodeBias, 
            const Mat3 &obsInf, bool updateNodeTarget, 
            Factor::robustFactorType robust_type):
        Factor(3,9, robust_type),
        dt_(dt), gyro_(gyroscope),W_(obsInf),
        J_(Mat<3,9>::Zero())
{
    bias_ = nodeBias->get_state();
    Robs_.exp(hat3((gyro_ - bias_) * dt_));

    // [0] -> Node Origin   size = 3
    // [1] -> Node Traget   size = 3
    // [2] -> Bias gyro     size = 3
    std::vector<uint_t>  nodes_ids;
    nodes_ids.push_back(nodeOrigin->get_id());
    nodes_ids.push_back(nodeTarget->get_id());
    nodes_ids.push_back(nodeBias->get_id());

    
    std::vector<uint_t> sizes_vector = {3,3,3};
    
    //function from factors.hpp, for factors that have several nodes connected tha require to maintain order, mostly for the Jacobian
    std::vector<uint_t> order = mrob::getJacobianIndexUnorderedNodes(nodes_ids, sizes_vector, jacobian_node_index_);

    
    std::vector<std::shared_ptr<Node> >  smart_pointers_vector = {nodeOrigin, nodeTarget, nodeBias};
    for (auto i : order)
    {
        neighbourNodes_.push_back(smart_pointers_vector[i]);
    }



    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        Mat3 RxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( RxOrigin * Robs_.R() );
    }
}



void FactorGyroBiasProp::evaluate_residuals()
{
    // From Origin we observe Target such that: R_o * R_obs = R_t
    // Rr = Rxo * Exp((w-b)*dt) * Rxt^-1

    Mat3 RxOrigin = get_neighbour_nodes()->at(0)->get_state();
    Mat3 RxTarget = get_neighbour_nodes()->at(1)->get_state();
    bias_ = get_neighbour_nodes()->at(2)->get_state();
    Robs_.exp(hat3((gyro_ - bias_) * dt_));
    Rr_ = SO3(RxOrigin) * Robs_ * SO3(RxTarget).inv();
    r_ = Rr_.ln_vee();
}

void FactorGyroBiasProp::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SO3 and small perturbations)
    Mat3 inv_left_jacobian, left_jacobian_bias;
    inv_left_jacobian = inv_left_jacobian_SO3(r_);
    J_.block<3,3>(0,jacobian_node_index_[0]) = inv_left_jacobian;
    J_.block<3,3>(0,jacobian_node_index_[1]) = -inv_left_jacobian*Rr_.adj();
    left_jacobian_bias = left_jacobian_SO3((gyro_ - bias_) * dt_);
    Mat3 RxOrigin = get_neighbour_nodes()->at(0)->get_state();
    J_.block<3,3>(0,jacobian_node_index_[2])= -dt_*inv_left_jacobian*RxOrigin*left_jacobian_bias;
}


void FactorGyroBiasProp::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void FactorGyroBiasProp::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Robs_.R()
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Nodes " << neighbourNodes_.size()
            << std::endl;
}
