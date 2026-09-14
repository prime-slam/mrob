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
 * factorCameraProj3dPointSim3.cpp
 *
 *  Created on: September 1, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factorCameraProj3dPointSim3.hpp"
#include <iostream>

using namespace mrob;

FactorCameraProj3dPointSim3::FactorCameraProj3dPointSim3(const Mat21 &observation, std::shared_ptr<Node> &nodePose,
                std::shared_ptr<Node> &nodeLandmark,
                const Mat41 &camera_k,
                const Mat2 &obsInf,
                Factor::robustFactorType robust_type):
            Factor(2,10,robust_type), obs_(observation), camera_k_(camera_k), W_(obsInf), reversedNodeOrder_(false)
{
    // chek for order, we need to ensure id_0 < id_1
    if (nodePose->get_id() < nodeLandmark->get_id())
    {
        neighbourNodes_.push_back(nodePose);
        neighbourNodes_.push_back(nodeLandmark);
    }
    else
    {
        neighbourNodes_.push_back(nodeLandmark);
        neighbourNodes_.push_back(nodePose);
        // set reverse mode
        reversedNodeOrder_ = true;
    }
    r_.setZero();
    J_.setZero();
}

Mat21 FactorCameraProj3dPointSim3::project_point(const Mat31 point)
{
    Mat21 result = Mat21::Zero();
    // Check for an aberrration
    // TODO this should remove the poiunt in question from the estimation
    if (point[2] < 1e-6)
        return result;
    // fx * x /z + cx
    // fy * y /z + cy
    matData_t invz = 1.0 / point[2];
    result << camera_k_[0] * point[0] * invz + camera_k_[2],
              camera_k_[1] * point[1] * invz + camera_k_[3];
    return result;
}

void FactorCameraProj3dPointSim3::evaluate_residuals()
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
    r_ = project_point(local_point_) - obs_;

}
void FactorCameraProj3dPointSim3::evaluate_jacobians()
{
    /** This function is calculated by using the chain rule.
     *  Here, our convention of left-hand-side retraction of poses makes the derivatives more involved
     *  than comparing with taking a rhs convetion. 
     *  dr / dT = d proj_k / d p' * d p' / d T
     *      J_project:
     *      d proj_k / d p' = [fx/z  0   -fx/z^2 x]
     *                        [0    fy/z -fy/z^2 y]
     *      Jr:
     *      d p' / d T = d ( T-1 Exp(-dx) l ) / dl = T-1 [l^ -I -l]
     * 
     *  and
     * 
     *  dr / d l = d proj_k / d p' * d p' / d l
     *      d p' / d l = d (T^-1 l ) d l = sR  (from Tinv, from T is 1/sR')
    */
    Mat<4,7> Jr = Mat<4,7>::Zero();
    Jr.topLeftCorner<3,3>() = hat3(landmark_);
    Jr.block<3,3>(0,3) =  -Mat3::Identity();
    Jr.topRightCorner<3,1>() = -landmark_;
    Mat<2,3> J_project = Mat<2,3>::Zero();
    // Check for point in the image plane
    if (local_point_[2] < 1e-6)
    {
        J_.setZero(); //we can't really use the gradient, but increases the Ker() -> should be handled by LM.
        return;
    }
    matData_t invz = 1.0 / local_point_[2];
    J_project << camera_k_[0] * invz, 0, -camera_k_[0] * invz * invz * local_point_[0] ,
                 0, camera_k_[1] * invz, -camera_k_[1] * invz * invz * local_point_[1];
    if (reversedNodeOrder_)
    {
        J_.topLeftCorner<2,3>() = J_project * Tinv_.sR();
        J_.topRightCorner<2,7>() = J_project * ( Tinv_.T()* Jr).topLeftCorner<3,7>();
    }
    else
    {
        J_.topLeftCorner<2,7>() = J_project * ( Tinv_.T()* Jr).topLeftCorner<3,7>();
        J_.topRightCorner<2,3>() = J_project * Tinv_.sR(); 
    }
}

void FactorCameraProj3dPointSim3::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void FactorCameraProj3dPointSim3::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}


