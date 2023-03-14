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
 * factorCameraProj3dLine.cpp
 *
 *  Created on: March 14, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factorCameraProj3dLine.hpp"
#include <iostream>

using namespace mrob;

FactorCameraProj3dLine::FactorCameraProj3dLine(const Mat21 &obsPoint1,
                const Mat21 &obsPoint2,
                std::shared_ptr<Node> &nodePose,
                std::shared_ptr<Node> &nodePoint1,
                std::shared_ptr<Node> &nodePoint2,
                const Mat41 &camera_k,
                const Mat2 &obsInf,
                Factor::robustFactorType robust_type):
            Factor(1,12,robust_type), camera_k_(camera_k), W_(obsInf)
{
    // Usually, we check for the order of nodes, such that id_0 < id_1 < id_2
    // Since we have 3 nodes, we will not preserve the order and when creating the adjacency matrix in factor_graph_solve.cpp
    // the insert triplet function will just search for the correct order. This is a a Sparse mat Row,
    // so the search is only on the nodes considered in the factor
    // TODO: this could be sorted for a (minuscule?) speed increase.
    neighbourNodes_.push_back(nodePose);
    neighbourNodes_.push_back(nodePoint1);
    neighbourNodes_.push_back(nodePoint2);
    // calculate line obs
    line_obs_ = calculate_image_line(obsPoint1, obsPoint2);
    r_.setZero();
    J_.setZero();
}

Mat21 FactorCameraProj3dLine::project_point(const Mat31 point)
{
    Mat21 result = Mat21::Zero();
    // Check for an aberrration
    if (point[2] < 1e-6)
        return result;
    // fx * x /z + cx
    // fy * y /z + cy
    result << camera_k_[0] * point[0] / point[2] + camera_k_[2],
              camera_k_[1] * point[1] / point[2] + camera_k_[3];
    return result;
}

Mat31 FactorCameraProj3dLine::calculate_image_line(const Mat21 &p1, const Mat21 &p2)
{
    // This line is in homog coordinates
    // Points are pixel coordinates and this is the cross product of their homog coordinates
    Mat31 line = Mat31::Zero();
    line << p1[1] - p2[1],
           -p1[0] + p2[0],
            p1[0]*p2[1] - p1[1]*p2[0];
    line = line / line.norm();
    return line;
}

void FactorCameraProj3dLine::evaluate_residuals()
{
    Mat4 Tx = get_neighbour_nodes()->at(0)->get_state();
    Tinv_ = SE3(Tx).inv();
    point1_ = get_neighbour_nodes()->at(1)->get_state();
    local_point1_ = Tinv_.transform(point1_);
    point2_ = get_neighbour_nodes()->at(2)->get_state();
    local_point2_ = Tinv_.transform(point2_);
    r_  << project_point(local_point2_) ;

}
void FactorCameraProj3dLine::evaluate_jacobians()
{
    /** This function is calculated by using the chain rule.
     *  Here, our convention of left-hand-side retraction of poses makes the derivatives more involved
     *  than comparing with taking a rhs convetion. 
     *  dr / dT = d proj_k / d p' * d p' / d T
     *      d proj_k / d p' = [fx/z  0   -fx/z^2 x]
     *                        [0    fy/z -fy/z^2 y]
     *      d p' / d T = d ( T-1 Exp(-dx) l ) / dl = T-1 [l^ -I]
     * 
     *  and
     * 
     *  dr / d l = d proj_k / d p' * d p' / d l
     *      d p' / d l = d (T^-1 l ) d l = R'  (this is linear)
    */
    Mat<4,6> Jr = Mat<4,6>::Zero();
    Jr.topLeftCorner<3,3>() = hat3(local_point1_);
    Jr.topRightCorner<3,3>() =  -Mat3::Identity();
    Mat<2,3> J_project = Mat<2,3>::Zero();
    // Check for point in the image plane
    if (local_point1_[2] < 1e-6  || local_point2_[2] < 1e-6)
    {
        J_.setZero(); //we can't really use the gradient, but increases the Ker() -> should be handled by LM.
        //message invalid line
        return;
    }
    J_project << camera_k_[0] / local_point_[2], 0, -camera_k_[0] / (local_point_[2]*local_point_[2]) * local_point_[0] ,
                 0, camera_k_[1] / local_point_[2], -camera_k_[1] / (local_point_[2]*local_point_[2]) * local_point_[1];
    
    // Joint Jacobian
    J_.topLeftCorner<2,3>() = J_project * Tinv_.R();
    J_.topRightCorner<2,6>() = J_project * ( Tinv_.T()* Jr).topLeftCorner<3,6>();

}

void FactorCameraProj3dLine::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void FactorCameraProj3dLine::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}


