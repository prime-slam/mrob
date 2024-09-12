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
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factor1Pose2d_diff.hpp"

#include <iostream>

using namespace mrob;

Factor1Pose2d_diff::Factor1Pose2d_diff(const Mat31 &observation, std::shared_ptr<Node> &n1,
        const Mat3 &obsInf, DiffFactor::robustFactorType robust_type) :
        DiffFactor(3, 3, robust_type), obs_(observation), W_(obsInf), J_(Mat3::Zero())
{
    neighbourNodes_.push_back(n1);
    d2r_dx_dz_.reserve(3);
}

void Factor1Pose2d_diff::evaluate_jacobians()
{
    // Evaluate Jacobian
    J_ = Mat3::Identity();
}

void Factor1Pose2d_diff::evaluate_residuals()
{
    r_ = get_neighbour_nodes()->at(0).get()->get_state() - obs_;
    r_(2) = wrap_angle(r_(2));
}

void Factor1Pose2d_diff::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Pose2d_diff::evaluate_dr_dz()
{
    dr_dz_.setIdentity(3,3);
    dr_dz_ *= -1;
}

MatRefConst Factor1Pose2d_diff::get_dr_dz() const
{
    return dr_dz_;
}

void Factor1Pose2d_diff::print() const
{
    std::cout << "Printing DiffFactor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}