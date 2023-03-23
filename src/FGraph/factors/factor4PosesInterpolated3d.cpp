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
 * factor4PosesInterpolated3d.cpp
 *
 *  Created on: Dec 1, 2022
 *      Author: Kazii Botashev
 *              kazii.botashev@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor4PosesInterpolated3d.hpp"

#include <iostream>

using namespace mrob;

Mat61 ComputeTdelta(SE3 Ta, SE3 Tb, float time)
{
    Mat61 xi_delta = (Tb * Ta.inv()).ln_vee() * time;
    return xi_delta;
}

SE3 InterpolatePose(SE3 Ta, SE3 Tb, float time)
{   
    Mat61 xi_delta = ComputeTdelta(Ta, Tb, time);
    SE3 Tt = SE3(xi_delta) * Ta;
    return Tt;
}

Factor4PosesInterpolated3d::Factor4PosesInterpolated3d(const Mat4 &observation, 
            std::shared_ptr<Node> &nodeOriginFirstPair, std::shared_ptr<Node> &nodeTargetFirstPair, 
            std::shared_ptr<Node> &nodeOriginSecondPair, std::shared_ptr<Node> &nodeTargetSecondPair, 
            const float time_first, const float time_second, const Mat6 &obsInf, 
            Factor::robustFactorType robust_type):
        Factor(6,24,robust_type), Tobs_(observation), W_(obsInf), time_origin(time_first), time_target(time_second)
{   // We have 5 different intersection posibilities for interpolation intervals.
    if ((nodeOriginFirstPair->get_id() < nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() < nodeOriginSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        nodes_intersection_case = 1;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 2;
        id_b_1 = 3;
    }
    else if ((nodeOriginFirstPair->get_id() < nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() == nodeOriginSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        nodes_intersection_case = 2;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 1;
        id_b_1 = 2;
    }
    else if ((nodeOriginFirstPair->get_id() == nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() == nodeTargetSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 3;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 0;
        id_b_1 = 1;
        if (time_second < time_first)
        {
            Tobs_.inv();
        }
    }
    else if ((nodeOriginSecondPair->get_id() < nodeOriginFirstPair->get_id()) && (nodeOriginFirstPair->get_id() == nodeTargetSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 4;
        Tobs_.inv();
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 1;
        id_b_1 = 2;
    }
    else if ((nodeOriginSecondPair->get_id() < nodeOriginFirstPair->get_id()) && (nodeTargetSecondPair->get_id() < nodeOriginFirstPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 5;
        Tobs_.inv();
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 2;
        id_b_1 = 3;
    }
}

Factor4PosesInterpolated3d::Factor4PosesInterpolated3d(const SE3 &observation, 
            std::shared_ptr<Node> &nodeOriginFirstPair, std::shared_ptr<Node> &nodeTargetFirstPair, 
            std::shared_ptr<Node> &nodeOriginSecondPair, std::shared_ptr<Node> &nodeTargetSecondPair, 
            const float time_first, const float time_second, const Mat6 &obsInf,
            Factor::robustFactorType robust_type):
        Factor(6,24, robust_type), Tobs_(observation), W_(obsInf), time_origin(time_first), time_target(time_second)
{
    if ((nodeOriginFirstPair->get_id() < nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() < nodeOriginSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        nodes_intersection_case = 1;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 2;
        id_b_1 = 3;
    }
    else if ((nodeOriginFirstPair->get_id() < nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() == nodeOriginSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        nodes_intersection_case = 2;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 1;
        id_b_1 = 2;
    }
    else if ((nodeOriginFirstPair->get_id() == nodeOriginSecondPair->get_id()) && (nodeTargetFirstPair->get_id() == nodeTargetSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 3;
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 0;
        id_b_1 = 1;
        if (time_second < time_first)
        {
            Tobs_.inv();
        }
    }
    else if ((nodeOriginSecondPair->get_id() < nodeOriginFirstPair->get_id()) && (nodeOriginFirstPair->get_id() == nodeTargetSecondPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 4;
        Tobs_.inv();
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 1;
        id_b_1 = 2;
    }
    else if ((nodeOriginSecondPair->get_id() < nodeOriginFirstPair->get_id()) && (nodeTargetSecondPair->get_id() < nodeOriginFirstPair->get_id()))
    {
        neighbourNodes_.push_back(nodeOriginSecondPair);
        neighbourNodes_.push_back(nodeTargetSecondPair);
        neighbourNodes_.push_back(nodeOriginFirstPair);
        neighbourNodes_.push_back(nodeTargetFirstPair);
        nodes_intersection_case = 5;
        Tobs_.inv();
        id_a_0 = 0;
        id_b_0 = 1;
        id_a_1 = 2;
        id_b_1 = 3;
    }
}


void Factor4PosesInterpolated3d::evaluate_residuals()
{
    // From interpolated origin we observe interpolated Target such that: T_o_ct * T_obs = T_t_ct
    // Tr = Txo_ct * Tobs * Txt_ct^-1
    Mat4 T_a_0 = get_neighbour_nodes()->at(id_a_0)->get_state();
    Mat4 T_b_0 = get_neighbour_nodes()->at(id_b_0)->get_state();
    Mat4 T_a_1 = get_neighbour_nodes()->at(id_a_1)->get_state();
    Mat4 T_b_1 = get_neighbour_nodes()->at(id_b_1)->get_state();
    SE3 TxOrigin = InterpolatePose(SE3(T_a_0), SE3(T_b_0), time_origin);
    SE3 TxTarget = InterpolatePose(SE3(T_a_1), SE3(T_b_1), time_target);
    Tr_ = TxOrigin * Tobs_ * TxTarget.inv();
    r_ = Tr_.ln_vee();

}
void Factor4PosesInterpolated3d::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat4 T_a_0 = get_neighbour_nodes()->at(id_a_0)->get_state();
    Mat4 T_b_0 = get_neighbour_nodes()->at(id_b_0)->get_state();
    Mat4 T_a_1 = get_neighbour_nodes()->at(id_a_1)->get_state();
    Mat4 T_b_1 = get_neighbour_nodes()->at(id_b_1)->get_state();
    Mat61 xi_delta_source = ComputeTdelta(SE3(T_a_0), SE3(T_b_0), time_origin);
    Mat61 xi_delta_target = ComputeTdelta(SE3(T_a_1), SE3(T_b_1), time_target);
    Mat6 J_T_a_0 = (1 - time_origin) * SE3(xi_delta_source).adj();
    Mat6 J_T_b_0 = time_origin * Mat6::Identity();
    Mat6 J_T_a_1 = (time_target - 1) * Tr_.adj() * SE3(xi_delta_target).adj();
    Mat6 J_T_b_1 = - time_target * Tr_.adj();
    if ((nodes_intersection_case == 1) || (nodes_intersection_case == 5))
    {
        J_ << J_T_a_0, J_T_b_0, J_T_a_1, J_T_b_1;
    }
    else if ((nodes_intersection_case == 2) || (nodes_intersection_case == 4))
    {
        J_ << J_T_a_0, J_T_b_0 + J_T_a_1, J_T_b_1, Mat6::Zero();
    }
    else if (nodes_intersection_case == 3)
    {
        J_ << J_T_a_0 + J_T_a_1, J_T_b_0 + J_T_b_1, Mat6::Zero(), Mat6::Zero();
    }
}

void Factor4PosesInterpolated3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor4PosesInterpolated3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Tobs_.T()
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << ", " << neighbourNodes_[1]->get_id()
              << std::endl;
}


