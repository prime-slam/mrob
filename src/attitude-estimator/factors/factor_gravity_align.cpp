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
* FactorGravityAlign.hpp
*
*  Created on: Jul 18, 2025
*      Author: Ivan Kakurin
*              i.kakurin@skoltech.ru
*              Gonzalo Ferrer
*              g.ferrer@skoltech.ru
*              Mobile Robotics Lab, Skoltech 
*/


#include "mrob/factors/factor_gravity_align.hpp"

#include <iostream>

using namespace mrob;


FactorGravityAlign::FactorGravityAlign(const Mat31 &acc, const Mat31 &grav, std::shared_ptr<Node> &n1,
            const Mat3 &obsInf, Factor::robustFactorType robust_type):
            Factor(3,3, robust_type), a_(acc.normalized()), g_(grav), W_(obsInf), J_(Mat3::Zero())
{
    // Ordering here is not a problem, the node is unique
    neighbourNodes_.push_back(n1);
}

FactorGravityAlign::FactorGravityAlign(const Mat31 &acc, std::shared_ptr<Node> &n1,
            const Mat3 &obsInf, Factor::robustFactorType robust_type):
            Factor(3,3, robust_type), a_(Mat31::Zero()), W_(obsInf), J_(Mat3::Zero())
{
    // Ordering here is not a problem, the node is unique
    g_ << 0.0, 0.0, 1.0;           // gravity default direction = [0, 0, 1]
    neighbourNodes_.push_back(n1);
}

void FactorGravityAlign::evaluate_residuals()
{
    // Anchor residuals as r = x - obs
    // r = X * a - g

    Mat3 x = get_neighbour_nodes()->at(0).get()->get_state();
    Ra_ = SO3(x).R() * a_;
    r_ = Ra_ - g_;
}

void FactorGravityAlign::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SO3 and small perturbations)
    // g0(X) = X * a - grav
    // J = dg0/dX = d(X * a - grav)/dX = d(X*a)/dX = dg/dX
    // J = d/dxi g(exp(xi)*X) = d/dxi (exp(xi) * X * a) = G * X * a = [(X * a)^]
    J_ = -hat3(Ra_);
}

/*
Factor 1posePoint2Plane

void Factor1PosePoint2Plane::evaluate_residuals()
{
    // r = <pi, Tp>
    Mat4 Tx = get_neighbour_nodes()->at(0)->get_state();
    SE3 T = SE3(Tx);
    Tx_ = T.transform(z_point_x_);
    r_ = Mat1(z_normal_y_.dot(Tx_ - z_point_y_));
}

void Factor1PosePoint2Plane::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat<3,6> Jr;
    Jr << -hat3(Tx_) , Mat3::Identity();
    J_ = z_normal_y_.transpose() * Jr;
}

*/

void FactorGravityAlign::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void FactorGravityAlign::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << a_
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Nodes " << neighbourNodes_.size()
            << std::endl;
}
