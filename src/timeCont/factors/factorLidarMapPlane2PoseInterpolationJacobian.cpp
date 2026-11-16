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
 * factorLidarMapPlane2PoseInterpolationJacobian.cpp
 *
 *  Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Gonzalo Ferrer
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factorLidarMapPlane2PoseInterpolationJacobian.hpp"
#include <iostream>
using namespace mrob;

FactorLidarMapPlane2PoseInterpolationJacobian::FactorLidarMapPlane2PoseInterpolationJacobian(
        const Mat41 &observation,
        const Mat31 &plane_point,
        const Mat31 &normal,
        std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget,
        const Mat4 &offset_lidar_imu,
        matData_t information,
        Factor::robustFactorType robust_type):

        // the base class stores q as its "map point" and computes r_ = p_w - q and its Jacobian (3 x 18)
        FactorLidarMap2PoseInterpolationJacobian(observation, plane_point, nodeOrigin, nodeTarget, offset_lidar_imu,
                                                 Mat3::Identity(), robust_type),
        normal_(normal.normalized())
{
    set_dim_obs(1);
    W1_(0,0) = information;
    r1_.setZero();
    J1_.setZero();
}

void FactorLidarMapPlane2PoseInterpolationJacobian::evaluate_residuals()
{
    FactorLidarMap2PoseInterpolationJacobian::evaluate_residuals();   // r_ = p_w - q
    r1_(0,0) = normal_.dot(r_);
}

void FactorLidarMapPlane2PoseInterpolationJacobian::evaluate_jacobians()
{
    FactorLidarMap2PoseInterpolationJacobian::evaluate_jacobians();   // J_ = J_p [M1 M2]   (3 x 18)
    J1_ = normal_.transpose() * J_;                                  // n^T J_p [M1 M2]
}

void FactorLidarMapPlane2PoseInterpolationJacobian::evaluate_chi2()
{
    chi2_ = 0.5 * W1_(0,0) * r1_(0,0) * r1_(0,0);
}

void FactorLidarMapPlane2PoseInterpolationJacobian::print() const
{
    std::cout << "FactorLidarMapPlane2PoseInterpolationJacobian   id = " << id_ << std::endl;
    std::cout << "obs_: " << obs_.transpose() << std::endl;
    std::cout << "plane point: " << map_point_.transpose() << "  normal: " << normal_.transpose() << std::endl;
    std::cout << "r: " << r1_ << "  W: " << W1_ << std::endl;
    std::cout << "J: " << J1_ << std::endl;
    std::cout << "chi2_: " << chi2_ << std::endl;
}
