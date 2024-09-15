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
 * factor2Poses3dTwistMatching.cpp
 *
 *  Created on: September 11, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3dTwistMatching.hpp"

#include <iostream>

using namespace mrob;


Factor2Poses3dTwistMatching::Factor2Poses3dTwistMatching(const Mat61 &observation, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat9 &obsInf, 
        Factor::robustFactorType robust_type):
        Factor(9,18,robust_type), obs_(observation), W_(obsInf), delta_t_(0.0)
{
    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);
    }
}



void Factor2Poses3dTwistMatching::evaluate_residuals()
{
    // From Origin we observe 
    Mat5 state_origin;
    state_origin = get_neighbour_nodes()->at(0)->get_state();
    Tx_origin_inv_ = SE3tc(state_origin).inv();
    Mat5 state_target = get_neighbour_nodes()->at(1)->get_state();
    SE3tc Tx_target(state_target);
    delta_t_ = get_neighbour_nodes()->at(1)->get_time_stamp() - 
               get_neighbour_nodes()->at(0)->get_time_stamp();
    
    SE3tc dT = Tx_origin_inv_ * Tx_target;
    //delta_t_ = dT.t();//XXX time is not well set in the matrices... need to check this, it should be in one place only!
    Mat91 twist;
    twist = dT.Ln();
    omega_ = twist.head(3);
    r_.head(3) =  omega_ - obs_.head(3);
    r_.tail(3) =  twist.tail(3) - obs_.tail(3);
    r_.segment<3>(3) = twist.segment<3>(3); // 0 observation
    //std::cout << "resuldula \n" << r_ << std::endl;
}
void Factor2Poses3dTwistMatching::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    J_.topRightCorner<9,9>() =  Tx_origin_inv_.adj().topLeftCorner<9,9>();
    J_.topLeftCorner<9,9>() = - J_.topRightCorner<9,9>();
}

void Factor2Poses3dTwistMatching::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3dTwistMatching::print() const
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

