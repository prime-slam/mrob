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
 * factor1Pose1LandmarkPoint2PlaneSim3.hpp
 *
 *  Created on: September 11, 2026
 *      Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTOR1POSE1LANDMARKPOINT2PLANESIM3_HPP_
#define FACTOR1POSE1LANDMARKPOINT2PLANESIM3_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/Sim3.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Point-to-plane of a world landmark expressed in a Sim3 camera frame.
 *
 * Nodes:
 *  - Sim3 pose T (camera-to-world, including per-frame scale)
 *  - Landmark3d X in world coordinates
 *
 * Observation:
 *  - local plane pi = [n', d]' in the camera frame (n is normalized)
 *
 * Residual:
 *   r = n' * T^{-1} X + d
 *
 * This is the constraint that lets a frame keep its rotation/translation
 * and absorb depth inconsistency into scale: shrinking/expanding T.s
 * moves the landmark along the camera ray relative to the local plane.
 *
 * Factor1LandmarkPoint2Plane4d remains valid when the plane comes from an
 * Eigen Factor in world coordinates (no pose Jacobian). Use this factor
 * when the plane measurement is local and the pose is NodeSim3.
 */
class Factor1Pose1LandmarkPoint2PlaneSim3 : public Factor
{
  public:
    Factor1Pose1LandmarkPoint2PlaneSim3(const Mat41 &observation,
            std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodeLandmark,
            const Mat1 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Pose1LandmarkPoint2PlaneSim3() = default;

    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;}

  protected:
    Mat41 obs_;
    Mat1 r_;
    Mat1 W_;
    Mat110 J_;
    bool reversedNodeOrder_;
    Sim3 Tinv_;
    Mat31 landmark_;
    Mat31 local_point_;
    Mat31 normal_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif /* FACTOR1POSE1LANDMARKPOINT2PLANESIM3_HPP_ */
