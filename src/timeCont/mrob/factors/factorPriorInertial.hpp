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
 * factorPriorInertial.hpp
 *
 *  Created on: Jan 20, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTORPRIORINERTIAL_HPP_
#define FACTORPRIORINERTIAL_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * factor prior intertial, includes nodes from:
 *  - Current pose (SE3vel) dim9
 *  - bias gyro dim3
 *  - bias dim3
 *  - gravity dim3
 * 
 * The factor simply provides a prior distribution, give an mean and a precision matrix (from previous optim iteration)
 */

class FactorPriorInertial : public Factor
{
  public:
    FactorPriorInertial(const Mat5 &observationPoseTarget,
            const Mat91 &obsBias,
            std::shared_ptr<Node> &nodePoseTarget,
            std::shared_ptr<Node> &nodeBiasGyro,
            std::shared_ptr<Node> &nodeBiasAcc,
            std::shared_ptr<Node> &nodeGravity,
            const Mat<18,18> &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorPriorInertial() = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return obs_bias_gyro_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J_;}

  protected:
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    Mat31 obs_bias_gyro_,obs_bias_acc_,obs_gravity_;
    Vect<18> r_;
    Mat<18,18> W_;//inverse of observation covariance (information matrix)
    Mat<18,18> J_;//Joint Jacobian
    double delta_t_;

    SE3tc Tobs_target_inv_, Tr_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FactorPriorInertial_HPP_ */
