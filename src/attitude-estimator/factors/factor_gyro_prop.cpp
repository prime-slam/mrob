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
* FactorGyroProp.hpp
*
*  Created on: Jul 18, 2025
*      Author: Ivan Kakurin
*              i.kakurin@skoltech.ru
*              Gonzalo Ferrer
*              g.ferrer@skoltech.ru
*              Mobile Robotics Lab, Skoltech 
*/


#include "mrob/factors/factor_gyro_prop.hpp"

#include <iostream>

using namespace mrob;

FactorGyroProp::FactorGyroProp(const Mat31 &gyroscope, const double &dt, std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget, 
            Factor::robustFactorType robust_type):
            Factor(3,6, robust_type), W_(obsInf), J_(Mat<3,6>::Zero())
{
    Robs_.exp(hat3(gyroscope * dt));

    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);

        // inverse observations to correctly modify this
        Robs_ = Robs_.inv();
    }

    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // carefull on the reference frame that Robs is expressed at the X_origin frame, hence this change:
        Mat3 RxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( RxOrigin * Robs_.R() );
    }
}

FactorGyroProp::FactorGyroProp(const Mat31 &d_fi, std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget, 
            Factor::robustFactorType robust_type):
            Factor(3,6, robust_type), W_(obsInf), J_(Mat<3,6>::Zero())
{
    Robs_.exp(hat3(d_fi));
    
    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);

        // inverse observations to correctly modify this
        Robs_ = Robs_.inv();
    }

    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // carefull on the reference frame that Robs is expressed at the X_origin frame, hence this change:
        Mat3 RxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( RxOrigin * Robs_.R() );
    }
}

FactorGyroProp::FactorGyroProp(const Mat3 &d_R, std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget, 
            Factor::robustFactorType robust_type):
            Factor(3,6, robust_type), Robs_(d_R), W_(obsInf), J_(Mat<3,6>::Zero())
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

        // inverse observations to correctly modify this
        Robs_ = Robs_.inv();
    }

    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // carefull on the reference frame that Robs is expressed at the X_origin frame, hence this change:
        Mat3 RxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( RxOrigin * Robs_.R() );
    }
}


void FactorGyroProp::evaluate_residuals()
{
    // From Origin we observe Target such that: R_o * R_obs = R_t
    // Rr = Rxo * Robs * Rxt^-1

    Mat3 RxOrigin = get_neighbour_nodes()->at(0)->get_state();
    Mat3 RxTarget = get_neighbour_nodes()->at(1)->get_state();
    Rr_ = SO3(RxOrigin) * Robs_ * SO3(RxTarget).inv();
    r_ = Rr_.ln_vee();
}

void FactorGyroProp::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SO3 and small perturbations)
    Mat3 inv_left_jacobian;
    inv_left_jacobian = inv_left_jacobian_SO3(r_);
    J_.topLeftCorner<3,3>() = inv_left_jacobian;
    J_.topRightCorner<3,3>() = -inv_left_jacobian*Rr_.adj();
    //J_.topRightCorner<3,3>() = -Rr_.adj()*inv_left_jacobian; //Ivan: the right solutions should be the above
}


void FactorGyroProp::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void FactorGyroProp::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Robs_.R()
            << "\n Residuals= \n" << r_
            << " \nand Information matrix\n" << W_
            << "\n Calculated Jacobian = \n" << J_
            << "\n Chi2 error = " << chi2_
            << " and neighbour Nodes " << neighbourNodes_.size()
            << std::endl;
}
