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
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
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

void mrob::Factor1Pose2d_diff::evaluate_dr_dz()
{
    dr_dz_ = - Mat3::Identity();
}

void mrob::Factor1Pose2d_diff::evaluate_d2r_dx_dz()
{
    d2r_dx_dz_[0].setZero();
    d2r_dx_dz_[1].setZero();
    d2r_dx_dz_[2].setZero();
    std::cout << "mrob::Factor1Pose2d_diff::evaluate_d2r_dx_dz - not implemented!" << std::endl;
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
