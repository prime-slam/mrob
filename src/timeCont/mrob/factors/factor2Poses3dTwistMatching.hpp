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
 * factor2Poses3dTwistMatching.hpp
 *
 *  Created on: Sep 11, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR2POSES3DTWISTMATCHING_HPP_
#define FACTOR2POSES3DTWISTMATCHING_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/SE3.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * factor 2 poses 3d twsit matching creates a high dimension interpolation between poses of constant omega and acceleration
 * and matches it with IMU observations.
 */

class Factor2Poses3dTwistMatching : public Factor
{
  public:
    Factor2Poses3dTwistMatching(const Mat61 &observation, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat9 &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor2Poses3dTwistMatching() = default;
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

    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J_;}

  protected:
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    // being [0]->J_origin and [1]->J_target
    // declared here but initialized on child classes
    Mat61 obs_; // obs = [omega, acc]
    Mat91 r_; //and residuals [omega, vel, acc]
    Mat9 W_;//inverse of observation covariance (information matrix)
    Mat<9,18> J_;//Joint Jacobian
    double delta_t_;
    Mat31 omega_;

    SE3tc Tx_target_inv_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3DTWISTMATCHING_HPP_ */
