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
 *  Created on: Oct 31, 2025
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factor2Bias3d.hpp"

#include <iostream>

using namespace mrob;

Factor2Bias3d::Factor2Bias3d(const Mat31 &observation, std::shared_ptr<Node> &n1, std::shared_ptr<Node> &n2,
        const Mat3 &obsInf, Factor::robustFactorType robust_type) :
        Factor(3, 6, robust_type), obs_(observation), W_(obsInf), J_(Mat<3,6>::Zero())
{
    
    if (n1->get_id() < n2->get_id())
    {
        neighbourNodes_.push_back(n1);
        neighbourNodes_.push_back(n2);
    }
    else
    {
        neighbourNodes_.push_back(n2);
        neighbourNodes_.push_back(n1);
        obs_ = -obs_;
    }
}


void Factor2Bias3d::evaluate_residuals()
{
    r_ = get_neighbour_nodes()->at(0).get()->get_state() - get_neighbour_nodes()->at(1).get()->get_state() - obs_;
}

void Factor2Bias3d::evaluate_jacobians()
{
    // Evaluate Jacobian
    J_.topLeftCorner<3,3>() = Mat3::Identity();
    J_.topRightCorner<3,3>()  = -Mat3::Identity();
}


void Factor2Bias3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor2Bias3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}


