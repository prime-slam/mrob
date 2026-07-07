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
 * factor1Pose3d_diff.hpp
 *
 *  Created on: Feb 26, 2025
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTOR1POSE3D_DIFF_HPP_
#define FACTOR1POSE3D_DIFF_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor_diff.hpp"

namespace mrob{

/**
 * The Factor1Poses3d is a vertex representing the distribution
 * of a nodePose3d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 *
 * In particular, the residual of this factor is:
 *   r = (x - obs) = Tx * Tobs^{-1}
 */

class Factor1Pose3d_diff : public DiffFactor
{
  public:
    Factor1Pose3d_diff(const Mat4 &observation, std::shared_ptr<Node> &n1, const Mat6 &obsInf,
            DiffFactor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    Factor1Pose3d_diff(const SE3 &observation, std::shared_ptr<Node> &n1, const Mat6 &obsInf,
            DiffFactor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Pose3d_diff() = default;
    /**
     * Returns the chi2 error and fills the residual vector
     */
    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;
    void evaluate_dr_dz() override;
    void print() const override;

    MatRefConst get_obs() const override {return Tobs_.T();};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};
    MatRefConst get_dr_dz() const override {return dr_dz_;};


  protected:
    Mat61 r_; //and residuals
    SE3 Tobs_, Tr_;//Transformation for the observation and the residual
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat6 J_;//Jacobian
    Mat6 dr_dz_; //derivative of residuals with reference to observations


};


}



#endif /* FACTOR1POSE3D_DIFF_HPP_ */
