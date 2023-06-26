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
 * diffFactor1PosePoint2Point.cpp
 *
 *  Created on: May 28, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/diffFactor1PosePoint2Point.hpp"

#include <iostream>


using namespace mrob;

DiffFactor1PosePoint2Point::DiffFactor1PosePoint2Point(const Mat31 &z_point_x, const Mat31 &z_point_y,  std::shared_ptr<Node> &node,
            const Mat3 &obsInf, Factor::robustFactorType robust_type):
        Factor1PosePoint2Point(z_point_x,z_point_y, node, obsInf, robust_type)
{
    // 
}

DiffFactor1PosePoint2Point::~DiffFactor1PosePoint2Point() = default;


MatRefConst DiffFactor1PosePoint2Point::calculate_derivative_obs_state()
{
    // recalculates the jacobians and provides the derivative d2r / dx /dz
    // 1) calculates the residuals from parent class Factor1PosePoint2Point
    Factor1PosePoint2Point::evaluate_residuals();

    //2) calculate jacobian (it will be used later)
    Factor1PosePoint2Point::evaluate_jacobians();

    //3) calculate the new derivative, the matrix 
    derivative_dz_dT_.setZero();

    // 3.1) partial derivative wrt x, matrix shape 3x6
    // dC/dxdxi = d / dx ( dr'/dT * r) = dr'/dz * dr/dT + d2r'/dzdT * r
    //          = R*[-(Tx)^ I] 
    Mat4 Tnode = Factor1PosePoint2Point::get_neighbour_nodes()->at(0)->get_state();
    SE3 T = SE3(Tnode);
    // first orders are summed for the x observation
    derivative_dz_dT_.topLeftCorner<3,6>() = T.R().transpose() *J_;
    // second order part, it needs to be calcualted for each coordiante of x:
    Mat3 second_order_derivative = Mat3::Zero();
    for (uint_t i = 0; i < 3; ++i)
    {
        second_order_derivative.col(i) = r_.transpose() * hat3(T.R().col(i));
    }
    derivative_dz_dT_.topLeftCorner<3,3>() += second_order_derivative;

    // 3.2) partial derivative wrt y, matrix shape 3x6
    // dC/dydxi = d / dy ( dr'/dT * r) = (-I)*[-(Tx)^ I]
    derivative_dz_dT_.block<3,6>(3,0) = -J_;
    
    // 3.3) partial derivative wrt w, matrix shape 1x6
    // dC/dydxi = d / dw (w * r' *dr'/dT ) = r' * [-(Tx)^ I]
    derivative_dz_dT_.bottomLeftCorner<1,6>() = r_.transpose() * J_;

    // Note: minus is from the implicit optimziation that the gradient susbtract the solution.
    //We account for that in here
    return -derivative_dz_dT_;
}