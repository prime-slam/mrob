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

    xi_full_target_inv_origin_ = (T_origin.inv() * T_target).Ln();
    xi_target_inv_origin_ = thau_taw_ * xi_full_target_inv_origin_;
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
    // Assumes the residuals have been evaluated at the current state.
    //
    // r = R_tau p + p_tau - m, with T_tau = T1 B, B = Exp(tau xi), xi = Ln(A), A = T1^{-1} T2.
    // For T_tau -> T_tau Exp(delta_tau) (right perturbation):  dr/d delta_tau = J_p = [ -R_tau hat(p), R_tau, 0 ].
    // The interpolated pose depends on both nodes through B and through xi = Ln(A):
    //
    //   target T2 -> T2 Exp(d2):  Ln(A') = xi + Jr^{-1}(xi) d2
    //                             Exp(tau xi + tau Jr^{-1}(xi) d2) = B Exp(tau Jr(tau xi) Jr^{-1}(xi) d2)
    //                             =>  delta_tau = tau Jr(tau xi) Jr^{-1}(xi) d2
    //   origin T1 -> T1 Exp(d1):  A' = Exp(-d1) A  =>  Ln(A') = xi - Jl^{-1}(xi) d1
    //                             T1' B' = T1 B Exp(Ad_{B^-1} d1) Exp(-tau Jr(tau xi) Jl^{-1}(xi) d1)
    //                             =>  delta_tau = [ Ad_{B^-1} - tau Jr(tau xi) Jl^{-1}(xi) ] d1
    // (tau = 0: M1 = I, M2 = 0.  tau = 1: M1 = 0, M2 = I.)
    J_.setZero();

    const Mat3 R_taw = T_taw_.R();
    const matData_t tau = thau_taw_;

    Mat<3,9> J_point = Mat<3,9>::Zero();
    J_point.block<3,3>(0,0) = -R_taw * hat3(point_obs_imu_frame_);
    J_point.block<3,3>(0,3) =  R_taw;

    const SE3tc B_tau = SE3tc(xi_target_inv_origin_);
    const Mat<10,10> Ad_Binv = B_tau.inv().adj();
    const Mat<10,10> Jr_tau  = right_jacobian_tc(xi_target_inv_origin_);          // Jr(tau xi)
    const Mat<10,10> Jl_inv_xi = inv_left_jacobian_tc(xi_full_target_inv_origin_);  // Jl^{-1}(xi)
    const Mat<10,10> Jr_inv_xi = inv_right_jacobian_tc(xi_full_target_inv_origin_); // Jr^{-1}(xi)

    // The 10x10 products are formed first, and then restricted to the 9x9 time-free block.
    const Mat<10,10> M1 = Ad_Binv - tau * (Jr_tau * Jl_inv_xi);
    const Mat<10,10> M2 = tau * (Jr_tau * Jr_inv_xi);

    J_.block<3,9>(0,0) = J_point * M1.topLeftCorner<9,9>();
    J_.block<3,9>(0,9) = J_point * M2.topLeftCorner<9,9>();
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