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
 * factor2PosesPoint2PlaneInterpolated.cpp
 *
 *  Created on: Feb 24, 2023
 *      Author: Kazii Botashev
 *              kazii.botashev@skoltech.ru
 *              Mobile Robotics Lab.
 */

#include "mrob/factors/factor2PosesPoint2PlaneInterpolated.hpp"

#include <iostream>


using namespace mrob;

Mat61 ComputeTdelta(SE3 Ta, SE3 Tb, double time)
{
    Mat61 xi_delta = (Tb * Ta.inv()).ln_vee() * time;
    return xi_delta;
}

SE3 InterpolatePose(SE3 Ta, SE3 Tb, double time)
{   
    Mat61 xi_delta = ComputeTdelta(Ta, Tb, time);
    SE3 Tt = SE3(xi_delta) * Ta;
    return Tt;
}

factor2PosesPoint2PlaneInterpolated::factor2PosesPoint2PlaneInterpolated(const Mat31 &z_point_x, 
                                                                        const Mat31 &z_point_y, 
                                                                        const Mat31 &z_normal_y, 
                                                                        const double &weight,
                                                                        std::shared_ptr<Node> &begin_node, 
                                                                        std::shared_ptr<Node> &end_node, 
                                                                        const double &alpha_time,
                                                                        const Mat1 &obsInf, 
                                                                        Factor::robustFactorType robust_type):
                                                                        Factor(1,12, robust_type), 
                                                                        z_point_x_(z_point_x), 
                                                                        z_point_y_(z_point_y), 
                                                                        alpha_time_(alpha_time),
                                                                        Tx_(Mat31::Zero()), 
                                                                        z_normal_y_(z_normal_y), 
                                                                        weight_(weight), 
                                                                        r_(0.0), 
                                                                        W_(obsInf)
{
    neighbourNodes_.push_back(begin_node);
    neighbourNodes_.push_back(end_node);
    id_begin_node = 0;
    id_end_node = 1;
}


void factor2PosesPoint2PlaneInterpolated::evaluate_residuals()
{
    // r = <pi, Tp>
    Mat4 begin_T = get_neighbour_nodes()->at(id_begin_node)->get_state();
    Mat4 end_T = get_neighbour_nodes()->at(id_end_node)->get_state();
    SE3 T_alpha = InterpolatePose(SE3(begin_T), SE3(end_T), alpha_time_);
    Tx_ = T_alpha.transform(z_point_x_);
    r_ = weight_ * Mat1(z_normal_y_.dot(Tx_ - z_point_y_));
}

void factor2PosesPoint2PlaneInterpolated::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat4 begin_T = get_neighbour_nodes()->at(id_begin_node)->get_state();
    Mat4 end_T = get_neighbour_nodes()->at(id_end_node)->get_state();
    Mat61 xi_delta = ComputeTdelta(SE3(begin_T), SE3(end_T), alpha_time_);
    Mat6 J_begin_T = (1 - alpha_time_) * SE3(xi_delta).adj();
    Mat6 J_end_T = alpha_time_ * Mat6::Identity();
    Mat<6,12> Jxi;
    Jxi << J_begin_T, J_end_T;
    Mat<3,6> Jr;
    Jr << -hat3(Tx_) , Mat3::Identity();
    J_ = weight_ * z_normal_y_.transpose() * Jr * Jxi;
}

void factor2PosesPoint2PlaneInterpolated::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void factor2PosesPoint2PlaneInterpolated::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs point x= \n" << z_point_x_
              << "\nobs point y =\n" << z_point_y_
              << "\nobs normal y =\n" << z_normal_y_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}
