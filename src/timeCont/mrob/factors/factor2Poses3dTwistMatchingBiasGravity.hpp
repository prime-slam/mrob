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

#ifndef FACTOR2POSES3DTWISTMATCHINGBIASGRAVITY_HPP_
#define FACTOR2POSES3DTWISTMATCHINGBIASGRAVITY_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/SE3.hpp"
#include "mrob/factor.hpp"
#include <vector>

namespace mrob{

/**
 * factor 2 poses 3d twist matching creates a high dimension interpolation between poses of constant omega and acceleration
 * and matches it with IMU observations.
 * 
 * Node origin: Initial Pose
 * Node target: Final Pose
 * Node bias acc (landmark is a 3D vector)
 * Node bias gyr (landmark is a 3D vector)
 * Node gravity
 * 
 */

class Factor2Poses3dTwistMatchingBiasGravity : public Factor
{
  public:
    Factor2Poses3dTwistMatchingBiasGravity(const Mat61 &observation,
            std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, 
            std::shared_ptr<Node> &nodeBiasAcc, 
            std::shared_ptr<Node> &nodeBiasGyro,
            std::shared_ptr<Node> &nodeGravity, 
            const Mat9 &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor2Poses3dTwistMatchingBiasGravity() = default;
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
    Mat91 r_; //and residuals
    Mat101 xi_target_inv_origin_;
    Mat9 W_;//inverse of observation covariance (information matrix)
    Mat<9,27> J_;//Joint Jacobian 9+9+3+3+3
    double delta_t_;
    Mat31 omega_;

    SE3tc T_target_inv_origin_;

    std::vector<uint_t> jacobian_node_index_;
    // Map from original node position ([origin,target,bias_acc,bias_gyro,gravity])
    // to the sorted position inside neighbourNodes_ (sorted by node id).
    std::vector<uint_t> original_to_sorted_index_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3DTWISTMATCHINGBIASGRAVITY_HPP_ */
