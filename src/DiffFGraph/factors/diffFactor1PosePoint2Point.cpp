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
    Factor1PosePoint2Point::evaluate_residuals();//TODO fix ambiguity

    //2) calculate jacobian (it will be used later)
    Factor1PosePoint2Point::evaluate_jacobians();

    //3) calculate the new derivative
    Mat<3,6> derivative_dx_dz;
    
    return derivative_dx_dz;
}