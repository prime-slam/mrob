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
 * factor1Pose1LandmarkPoint2PlaneSim3.cpp
 *
 *  Created on: September 11, 2026
 *      Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factor1Pose1LandmarkPoint2PlaneSim3.hpp"
#include "mrob/SO3.hpp"

#include <iostream>

using namespace mrob;

Factor1Pose1LandmarkPoint2PlaneSim3::Factor1Pose1LandmarkPoint2PlaneSim3(
        const Mat41 &observation,
        std::shared_ptr<Node> &nodePose,
        std::shared_ptr<Node> &nodeLandmark,
        const Mat1 &obsInf,
        Factor::robustFactorType robust_type):
    Factor(1,10, robust_type),
    obs_(observation),
    W_(obsInf),
    reversedNodeOrder_(false)
{
    matData_t normal_norm = obs_.head<3>().norm();
    if (normal_norm > 0.0)
        obs_ /= normal_norm;

    if (nodePose->get_id() < nodeLandmark->get_id())
    {
        neighbourNodes_.push_back(nodePose);
        neighbourNodes_.push_back(nodeLandmark);
    }
    else
    {
        neighbourNodes_.push_back(nodeLandmark);
        neighbourNodes_.push_back(nodePose);
        reversedNodeOrder_ = true;
    }
    r_.setZero();
    J_.setZero();
}

void Factor1Pose1LandmarkPoint2PlaneSim3::evaluate_residuals()
{
    uint_t poseIndex = 0;
    uint_t landmarkIndex = 1;
    if (reversedNodeOrder_)
    {
        landmarkIndex = 0;
        poseIndex = 1;
    }
    Mat4 Tx = get_neighbour_nodes()->at(poseIndex)->get_state();
    Tinv_ = Sim3(Tx).inv();
    landmark_ = get_neighbour_nodes()->at(landmarkIndex)->get_state();
    local_point_ = Tinv_.transform(landmark_);
    normal_ = obs_.head<3>();
    r_ = Mat1(normal_.dot(local_point_) + obs_(3));
}

void Factor1Pose1LandmarkPoint2PlaneSim3::evaluate_jacobians()
{
    /**
     * Left retraction T' = Exp(dxi) T, so
     *   p' = T^{-1} Exp(-dxi) X
     * with the same Jr as FactorCameraProj3dPointSim3:
     *   Jr = [X^  -I  -X]
     *   dp / dT = (Tinv * Jr)[:3, :]
     *   dp / dX = Tinv.sR()
     *   dr = n' dp
     */
    Mat<4,7> Jr = Mat<4,7>::Zero();
    Jr.topLeftCorner<3,3>() = hat3(landmark_);
    Jr.block<3,3>(0,3) = -Mat3::Identity();
    Jr.topRightCorner<3,1>() = -landmark_;
    Mat17 Jp = normal_.transpose() * (Tinv_.T() * Jr).topLeftCorner<3,7>();
    Mat13 Jl = normal_.transpose() * Tinv_.sR();
    if (reversedNodeOrder_)
    {
        J_.leftCols<3>() = Jl;
        J_.rightCols<7>() = Jp;
    }
    else
    {
        J_.leftCols<7>() = Jp;
        J_.rightCols<3>() = Jl;
    }
}

void Factor1Pose1LandmarkPoint2PlaneSim3::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Pose1LandmarkPoint2PlaneSim3::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}
