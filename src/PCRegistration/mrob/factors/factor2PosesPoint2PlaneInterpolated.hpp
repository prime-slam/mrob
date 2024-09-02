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
 * the distribution of observations of a 3D pose from a pose that interpolated using two poses at
 * time delta_time. The purpose of this factors is to estimate
 * the relative transformation from two different sets of point clouds, given some planar constraaints.
 * Mainly, it will include a pointcloud from images (chessboard) after solving the PnP (external)
 * and any sensor providing depth.
 *
* Observations
 *  - Point x
 *  - Point y, normal n_y, from reference y
 *
 * State to estimate is 3D pose y^T_x.
 *
 * The residual, as a convention in the library is:
 *   r = f(x) = n_y' * (Tx - y) -> 0 if the relative point is in the plane.
 *
 * The Jacobian is then
 *  dr = n_y' [-(Tp)^, I] [(1 - delta_time_) * SE3(xi_delta).adj(), delta_time * I]
 *
 * TODO: correctly characterize the covariance, since this is a contribution from 2 rv
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