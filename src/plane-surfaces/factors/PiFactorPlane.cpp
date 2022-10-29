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
 * PiFactorPlane.cpp
 *
 *  Created on: Oct 26, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/PiFactorPlane.hpp"

#include <iostream>
#include <Eigen/Cholesky>
#include "mrob/SE3.hpp"

using namespace mrob;

PiFactorPlane::PiFactorPlane(const Mat4 &Sobservation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodePlane,
            Factor::robustFactorType robust_type):
                        Factor(4,10, robust_type), W_(Mat4::Identity()), reversedNodeOrder_(false)
{
    // To preserve the order when building the Adjacency matrix
    if (nodePose->get_id() < nodePlane->get_id())
    {
        neighbourNodes_.push_back(nodePose);
        neighbourNodes_.push_back(nodePlane);
    }
    else
    {
        neighbourNodes_.push_back(nodePlane);
        neighbourNodes_.push_back(nodePose);
        // set reverse mode
        reversedNodeOrder_ = true;
    }

    // calculate observation as sqrt(S)'
    Sobs_ = Sobservation.llt().matrixL().transpose();
    std::cout << "obs = \n" << Sobs_ << std::endl;
}

void PiFactorPlane::evaluate_residuals()
{
    uint_t poseIndex = 0;
    uint_t landmarkIndex = 1;
    if (reversedNodeOrder_)
    {
        landmarkIndex = 0;
        poseIndex = 1;
    }
    // r = srqt(S)' * T' * pi
    Mat4 Tx = get_neighbour_nodes()->at(poseIndex)->get_state();
    S_mul_T_transp_ = Sobs_ * Tx.transpose();
    plane_ = get_neighbour_nodes()->at(landmarkIndex)->get_state();
    // orientation of the plane here does not matter, which is a great improvement over classical factor node 4d
    r_ = S_mul_T_transp_ * plane_;
}

void PiFactorPlane::evaluate_jacobians()
{
    // dr/dxi = sqrt(S)'* T' * G' * pi = sqrt(S)'* T' * [normal^ | 0 0 0  ]
    //                                                   0 0 0   | normal']
    //dr/dpi = sqrt(S)'* T'
    Mat<4,6> Jx = Mat<4,6>::Zero();
    Mat31 normal = plane_.head(3);
    Jx.topLeftCorner<3,3>() = hat3(normal);
    Jx.bottomRightCorner<1,3>() =  normal;
    if (!reversedNodeOrder_)
    {
        J_.topLeftCorner<4,6>() = S_mul_T_transp_ * Jx;
        J_.topRightCorner<4,4>() = S_mul_T_transp_;
    }
    else
    {
        J_.topLeftCorner<4,4>() = S_mul_T_transp_;
        J_.topRightCorner<4,6>() = S_mul_T_transp_ * Jx;
    }

}

void PiFactorPlane::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(r_);// wieghting already included in the residual by sqrt(S)'
}

void PiFactorPlane::print() const
{
    std::cout << "Printing pi Factor Plane: " << id_ << ", sqrt(S)'(obs)= \n" << Sobs_
               << "\n Residuals= \n" << r_
               << "\n Calculated Jacobian = \n" << J_
               << "\n Chi2 error = " << chi2_
               << " and neighbour Nodes " << neighbourNodes_.size()
               << std::endl;
}


