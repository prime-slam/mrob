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
 * factorLidarMap2PoseInterpolationJacobian.cpp
 *
 *  Created on: Sep 14, 2025
 *      Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Gonzalo Ferrer 
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/factorLidarMap2PoseInterpolationJacobian.hpp"
#include <iostream>
using namespace mrob;

FactorLidarMap2PoseInterpolationJacobian::FactorLidarMap2PoseInterpolationJacobian(
        const Mat41 &observation,
        const Mat31 &map_point, 
        std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, 
        const Mat4 &offset_lidar_imu,
        const Mat3 &obsInf, 
        Factor::robustFactorType robust_type):

        Factor(3,18,robust_type), obs_(observation), W_(obsInf), map_point_(map_point), point_obs_imu_frame_(Mat31::Zero()),
        T_offset_lidar_imu_(offset_lidar_imu), thau_taw_(0.0)
{
    // Order must be preserved
    neighbourNodes_.push_back(nodeOrigin);
    neighbourNodes_.push_back(nodeTarget);

}




void FactorLidarMap2PoseInterpolationJacobian::interpolate_pose(SE3tc &T_origin, SE3tc &T_target,const matData_t &t_obs)
{
    matData_t t1 = T_origin.time();
    matData_t t2 = T_target.time();
    thau_taw_ = (t_obs - t1)/(t2-t1);

    xi_target_inv_origin_ = thau_taw_*(T_origin.inv()*T_target).Ln();
    T_taw_ = T_origin * SE3tc(xi_target_inv_origin_);
}
void FactorLidarMap2PoseInterpolationJacobian::evaluate_residuals()
{
    // From Origin we observe 
    Mat5 state_origin;
    state_origin = get_neighbour_nodes()->at(0)->get_state();
    SE3tc Tx_origin = SE3tc(state_origin);
    Mat5 state_target = get_neighbour_nodes()->at(1)->get_state();
    SE3tc Tx_target = SE3tc(state_target);
    matData_t t_obs = obs_(3);
    interpolate_pose(Tx_origin, Tx_target, t_obs);
    point_obs_imu_frame_ = T_offset_lidar_imu_.transform(obs_.head(3));
    Mat4 transf_taw = T_taw_.T().topLeftCorner<4,4>();
    SE3 T_taw_SE3 = SE3(transf_taw);
    r_ = T_taw_SE3.transform (point_obs_imu_frame_) - map_point_;
}

void FactorLidarMap2PoseInterpolationJacobian::evaluate_jacobians()
{
    J_.setZero();
    Mat3 R_taw = T_taw_.R();
    J_.block<3,3>(0,9) = -thau_taw_ * R_taw * hat3(point_obs_imu_frame_);
    J_.block<3,3>(0,9+3) =  thau_taw_ * R_taw;

    Mat<10,10> inverse_jacobian;
    inverse_jacobian = inv_right_jacobian_tc(xi_target_inv_origin_);
    Mat9 clip_inverse_jacobian;
    clip_inverse_jacobian = inverse_jacobian.topLeftCorner<9,9>();
    J_.block<3,9>(0,9) = J_.block<3,9>(0,9) * clip_inverse_jacobian;
}


void FactorLidarMap2PoseInterpolationJacobian::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}   

void FactorLidarMap2PoseInterpolationJacobian::print() const
{
    std::cout << "FactorLidarMap2PoseInterpolationJacobian   id = " << id_ << std::endl;
    std::cout << "obs_: " << obs_.transpose() << std::endl;
    std::cout << "map_point_: " << map_point_.transpose() << std::endl;
    std::cout << "r_: " << r_.transpose() << std::endl;
    std::cout << "W_: " << W_ << std::endl;
    std::cout << "J_: " << J_ << std::endl;
    std::cout << "chi2_: " << chi2_ << std::endl;
}