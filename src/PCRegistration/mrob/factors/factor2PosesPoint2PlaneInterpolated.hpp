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
 * factor2PosesPoint2PlaneInterpolated.hpp
 *
 *  Created on: Feb 24, 2023
 *      Author: Kazii Botashev
 *              kazii.botashev@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTOR2POSESPOINT2PLANEINTERPOLATED_HPP_
#define FACTOR2POSESPOINT2PLANEINTERPOLATED_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * factor2PosesPoint2PlaneInterpolated is a vertex in the Fgraph library that models
 * in-scan time-continuous point cloud mapping procedure. Having a map of 3D points and a
 * 3D pointcloud scanned by moving agent between two poses during time interval we want to map observations
 * to map and perform point to plane ICP.
 * Refer to "CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure" paper by Dellenbach et al. 
 * to see particular use case for that factor.
 *
* Observations
 *  - Point x captured at pose T_tau at time t_alpha while agent moving between begin pose T_a and end pose T_b (scan)
 *  - Point y, normal n_y, from reference y (map)
 *
 * State to estimate is two 3D poses T_a and T_b. T_alpha - pose interpolated at time t_alpha [0,1] for interval T_a->T_b
 *
 * The residual, as a convention in the library is:
 *   r = f(x) = n_y' * (T_tau * x - y) -> 0 if the relative point is in the plane.
 *
 * The Jacobian is then
 *  dr = n_y' [-(Tp)^, I] [(1 - t_alpha) * (T_b * T_a.inv()).ln_vee() * time).adj(), t_alpha * I]
 *
 */

class factor2PosesPoint2PlaneInterpolated: public Factor
{
  public:
    factor2PosesPoint2PlaneInterpolated(const Mat31 &z_point_x, 
                                        const Mat31 &z_point_y, 
                                        const Mat31 &z_normal_y, 
                                        const double &weight,
                                        std::shared_ptr<Node> &begin_node, 
                                        std::shared_ptr<Node> &end_node, 
                                        const double &alpha_time,
                                        const Mat1 &obsInf,
                                        Factor::robustFactorType robust_type = Factor::robustFactorType::CAUCHY);
    ~factor2PosesPoint2PlaneInterpolated() override = default;
    /**
     * Jacobians are not evaluated, just the residuals
     * r = <pi, T p>
     *
     * The transformation T is, thus, the transformation from the point reference to the plane reference
     */
    virtual void evaluate_residuals() override;
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const;

    MatRefConst get_obs() const override {return r_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const override {return J_;};

  protected:
    Mat31 z_point_x_, z_point_y_,  Tx_;
    Mat31 z_normal_y_;
    double alpha_time_;
    double weight_;
    double id_begin_node;
    double id_end_node;
    // the residual of the point projected to the plane:
    Mat1 r_;//residual, for convention, we will keep an eigen object for the scalar
    Mat1 W_;
    Mat<1,12> J_;
};

}

#endif /* FACTOR2POSESPOINT2PLANEINTERPOLATED_HPP_ */