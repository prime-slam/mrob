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
 * factorLidarMapPlane2PoseInterpolationJacobian.hpp
 *
 *  Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Gonzalo Ferrer
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTORLIDARMAPPLANE2POSEINTERPOLATIONJACOBIAN_HPP_
#define FACTORLIDARMAPPLANE2POSEINTERPOLATIONJACOBIAN_HPP_

#include "mrob/factors/factorLidarMap2PoseInterpolationJacobian.hpp"

namespace mrob {

/**
 * Point-to-plane variant of FactorLidarMap2PoseInterpolationJacobian.
 *
 * The map is represented locally by a plane (unit normal n, point q on the plane, e.g. the centroid of the
 * neighbouring map points). With p_w = R_tau p + p_tau the LiDAR point in the world frame at its own time,
 *
 *      r = n^T ( p_w - q )                 (scalar, dim = 1)
 *
 * so that r = n^T r_p2p of the point-to-point factor (map point -> q). The Jacobian is the projection of the
 * point-to-point Jacobian on n,
 *
 *      dr/d delta_tau = n^T J_p = [ (p x u)^T, u^T, 0 ],   u = R_tau^T n,
 *      J_T1 = (n^T J_p) M1,   J_T2 = (n^T J_p) M2,
 *
 * with the same interpolation matrices M1, M2 as the point-to-point factor, whose code is reused. n and q are
 * constants of the factor (like the map point of the base class). The information is a scalar.
 */
class FactorLidarMapPlane2PoseInterpolationJacobian : public FactorLidarMap2PoseInterpolationJacobian
{
public:
    /**
     * @param observation  [x y z t]: point in the LiDAR frame and its acquisition time
     * @param plane_point  point q on the plane (world frame)
     * @param normal       plane normal (world frame); normalised internally
     * @param information  scalar information of the distance r
     */
    FactorLidarMapPlane2PoseInterpolationJacobian(const Mat41 &observation, const Mat31 &plane_point, const Mat31 &normal,
            std::shared_ptr<Node> &nodeOrigin, std::shared_ptr<Node> &nodeTarget, const Mat4 &offset_lidar_imu,
            matData_t information,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorLidarMapPlane2PoseInterpolationJacobian() = default;

    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;
    void print() const override;
    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r1_;}
    MatRefConst get_information_matrix() const override {return W1_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J1_;}
    const Mat31& get_normal() const {return normal_;}

protected:
    Mat31 normal_;
    Mat<1,1> r1_;
    Mat<1,1> W1_;
    Mat<1,18> J1_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif /* FACTORLIDARMAPPLANE2POSEINTERPOLATIONJACOBIAN_HPP_ */
