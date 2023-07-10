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
 * Factor1Sim3Point2Point.cpp
 *
 *  Created on: July 10, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/factor1Sim3Point2Point.hpp"
#include "mrob/SE3.hpp"
#include "mrob/sim3.hpp"
#include <iostream>


using namespace mrob;

Factor1Sim3Point2Point::Factor1Sim3Point2Point(const Mat31 &z_point_x, const Mat31 &z_point_y,  std::shared_ptr<Node> &node,
            const Mat4 &obsInf, Factor::robustFactorType robust_type):
        Factor(4,7,robust_type), W_(obsInf)
{
    z_point_x_homog_ << z_point_x, 1.0;
    z_point_y_homog_ << z_point_y, 1.0;
    neighbourNodes_.push_back(node);
}

void Factor1Sim3Point2Point::evaluate_residuals()
{
    // r = Tx - y
    Mat4 Tnode = get_neighbour_nodes()->at(0)->get_state();
    Sim3 S = Sim3(Tnode);
    Tx_ = S.S() * z_point_x_homog_;
    r_ = Tx_ - z_point_y_homog_;
}

void Factor1Sim3Point2Point::evaluate_jacobians()
{
    J_.setZero();
    J_.topLeftCorner<3,6>() << -hat3(Tx_.head(3)) , Mat3::Identity();
    J_(3,6) = -1;
}

void Factor1Sim3Point2Point::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Sim3Point2Point::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs point x= \n" << z_point_x_homog_
              << "\nobs point y =\n" << z_point_y_homog_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}

