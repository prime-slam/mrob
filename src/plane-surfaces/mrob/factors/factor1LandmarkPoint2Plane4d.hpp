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
 * factor1LandmarkPoint2Plane4d.hpp
 */

#ifndef FACTOR1LANDMARKPOINT2PLANE4D_HPP_
#define FACTOR1LANDMARKPOINT2PLANE4D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor1LandmarkPoint2Plane4d constrains a 3D landmark point in world
 * coordinates to lie on the plane estimated by an Eigen Factor.
 *
 * State to estimate:
 *  - 3D landmark point X in world coordinates
 *
 * Auxiliary input:
 *  - Eigen Factor plane pi = [n', d]' in world coordinates
 *
 * Residual:
 *   r = pi' * [X; 1] = n' * X + d
 *
 * Jacobian:
 *   dr = n'
 */
class Factor1LandmarkPoint2Plane4d : public Factor
{
  public:
    Factor1LandmarkPoint2Plane4d(std::shared_ptr<Node> &nodeLandmark,
            std::shared_ptr<EigenFactor> &eigenFactorPlane, const Mat1 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1LandmarkPoint2Plane4d() = default;

    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return plane_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;}

  protected:
    std::shared_ptr<EigenFactor> eigenFactorPlane_;
    Mat31 landmark_;
    Mat41 plane_;
    Mat1 r_;
    Mat1 W_;
    Mat<1,3> J_;
};

}

#endif /* FACTOR1LANDMARKPOINT2PLANE4D_HPP_ */
