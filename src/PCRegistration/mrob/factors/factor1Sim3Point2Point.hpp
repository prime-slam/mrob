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
 * Factor1Sim3Point2Point.hpp
 *
 *  Created on: July 10, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTOR1SIM3POINT2POINT_HPP_
#define FACTOR1SIM3POINT2POINT_HPP_

#include "mrob/matrix_base.hpp"

#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor1Sim3Point2Point is a vertex in the Fgraph library that models
 * the distribution of observations of a 3D pose. The purpose of this factors is to estimate
 * the relative transformation from two different sets of point clouds, given some planar constraaints.
 *
 * Observations
 *  - Point x
 *  - Point y, from reference y
 *
 * State to estimate is Sim3 pose y^S_x.
 *
 * The residual, as a convention in the library is:
 *   r = f(x) = (Sx - y) \in P^3 (R^4).
 * The homogenous coordinate in the residual is necessary since there is a problem of scale
 * when transforming the points.
 *
 * The Jacobian is then
 *  dr = [-(Tx)^, I,  0]
 *       [ 0      0, -1]
 *
 */

class Factor1Sim3Point2Point : public Factor
{
  public:
    Factor1Sim3Point2Point(const Mat31 &z_point_x, const Mat31 &z_point_y,  std::shared_ptr<Node> &node,
            const Mat4 &obsInf = Mat4::Identity(), Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Sim3Point2Point() override = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     * J = dr/dxi = [-Tx^. I 0]
     *              [  0   0 -1]
     */
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const;

    MatRefConst get_obs() const override {return r_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override{return W_;};
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const override {return J_;};

  protected:
    Mat41 z_point_x_homog_, z_point_y_homog_, Tx_;
    Mat41 r_;
    Mat4 W_;
    Mat<4,7> J_;
};

}



#endif /* FACTOR1SIM3POINT2POINT_HPP_ */
