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
 * factor1PosePoint2Plane4d.hpp
 *
 * 
 */

#ifndef FACTOR1POSEPOINT2PLANE4D_HPP_
#define FACTOR1POSEPOINT2PLANE4D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor1PosePoint2Plane4d constrains a local 3D point observed from a pose to
 * lie on a plane represented in the world frame as pi = [n', d]'.
 *
 * Observations
 *  - local point x
 *  - world plane pi = [nx, ny, nz, d]
 *
 * State to estimate is the world pose T of the local frame.
 *
 * Residual:
 *   r = pi' * [T x; 1] = n' * (T x) + d
 *
 * Jacobian:
 *   dr = n' [-(T x)^, I]
 */
class Factor1PosePoint2Plane4d : public Factor
{
  public:
    Factor1PosePoint2Plane4d(const Mat31 &z_point_x, const Mat41 &z_plane,
            std::shared_ptr<Node> &node, const Mat1 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1PosePoint2Plane4d() = default;

    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return z_plane_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};

  protected:
    Mat31 z_point_x_;
    Mat41 z_plane_;
    Mat31 Tx_;
    Mat1 r_;
    Mat1 W_;
    Mat<1,6> J_;
};

}

#endif /* FACTOR1POSEPOINT2PLANE4D_HPP_ */
