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
* factor1SO3obs.hpp
*
*  Created on: Jul 18, 2025
*      Author: Ivan Kakurin
*              i.kakurin@skoltech.ru
*              Gonzalo Ferrer
*              g.ferrer@skoltech.ru
*              Mobile Robotics Lab, Skoltech 
*/


#include "mrob/factors/factor1SO3obs.hpp"

#include <iostream>

using namespace mrob;


Factor1SO3obs::Factor1SO3obs(const SO3 &observation, std::shared_ptr<Node> &n1,
            const Mat3 &obsInf, Factor::robustFactorType robust_type):
            Factor(3,3, robust_type), Robs_(observation), W_(obsInf), J_(Mat3::Zero())
{
    // Ordering here is not a problem, the node is unique
    neighbourNodes_.push_back(n1);
}

void Factor1SO3obs::evaluate_residuals()
{
    // Anchor residuals as r = x - obs
    // r = ln(X * Tobs^-1) = - ln(Tobs * x^-1)
    // NOTE Tobs is a global observation (reference identity)

    // change dimentsion to be 3x3 and T becomes R (rotations) Check out src/FGraph/factors/fator1Pose3d.cpp
    Mat3 x = get_neighbour_nodes()->at(0).get()->get_state();
    Rr_ = SO3(x) * Robs_.inv();
    r_ = Rr_.ln_vee();
}

void Factor1SO3obs::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SO3 and small perturbations)
    // J = d/dxi ln(T X-1 exp(-xi) (T X-1)-1)= - Adj_{T X-1} = - Adj(Tr)
    // J = d/dxi ln(exp(xi)X T-1  (T X-1)-1)= I
    J_ = Mat3::Identity();
    // J_ = inv_left_jacobian_SO3(r_)*Rr_.adj();
}

void Factor1SO3obs::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1SO3obs::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Robs_.R()
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Nodes " << neighbourNodes_.size()
            << std::endl;
}
