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
 * factor1Pose1Plane4d.hpp
 *
 *  Created on: Oct 10, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTOR1POSE1PLANE4D_HPP_
#define FACTOR1POSE1PLANE4D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor 1 Pose (3D) and plane 4d relates a pose with a plane observation, extracted from
 * segmented points in a point cloud.
 *
 *
 */

class Factor1Pose1Plane4d : public Factor
{
  public:
    Factor1Pose1Plane4d(const Mat41 &observation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodePlane, const Mat4 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Pose1Plane4d() = default;

    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates the Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return obs_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id*/) const override {return J_;};


  private:
    Mat41 obs_, r_; //residuals
    Mat<4,10> J_;//Jacobians dimensions obs x [plane(4) + pose(6)]
    Mat4 W_;//inverse of observation covariance (information matrix)
    bool reversedNodeOrder_;
    // intermediate variables to keep
    Mat41 plane_;
    Mat4 Tinv_transp_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

#endif /* FACTOR1POSE1PLANE4D_HPP_ */
