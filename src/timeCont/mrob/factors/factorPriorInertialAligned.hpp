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
 * factorPriorInertialAligned.hpp
 *
 *  Created on: Nov 16, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTORPRIORINERTIALALIGNED_HPP_
#define FACTORPRIORINERTIALALIGNED_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Prior on the Aligned (with gravity on Z) inertial state of a window: a Gaussian N(mean, Lambda^{-1}) over
 *  - Pose-velocity (SE3tc node, dim 9, right perturbation T Exp(delta), time of the update = 0)
 *  - bias gyro  (R^3)
 *  - bias acc   (R^3)

 * i.e. 15 degrees of freedom. The information matrix Lambda is full (15x15); it is typically the
 * Schur complement of the previous window (see src/core/marginalization.hpp in the SLAM application), or a
 * diagonal matrix for the very first window.
 *
 * Residual (right error, "r = h(x) - z" with z the prior mean):
 *      r = [ Ln_9( Tbar^{-1} T )           (9)
 *            bg - bg_bar                   (3)
 *            ba - ba_bar                   (3)
 *
 * Jacobian (exact):
 *      J = blkdiag( Jr^{-1}(r_pose)_{9x9},  I_3,  I_3  )
 * which equals the identity at the prior mean. The Jacobian of the pose block is NOT the identity away
 * from the mean, and the left-error form Ln(T Tbar^{-1}) used previously has the adjoint of Tbar as
 * Jacobian, which couples the velocity to the position through the absolute node time.
 *
 * The nodes must be passed in this order: [pose, bias gyro, bias acc, gravity] (the Jacobian
 * columns follow it).
 */

class FactorPriorInertialAligned : public Factor
{
  public:
    FactorPriorInertialAligned(const Mat5 &observationPoseTarget,
            const Mat61 &obsBias,   // [bias gyro; bias acc;]
            std::shared_ptr<Node> &nodePoseTarget,
            std::shared_ptr<Node> &nodeBiasGyro,
            std::shared_ptr<Node> &nodeBiasAcc,
            const Mat<15,15> &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorPriorInertialAligned() = default;
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
    Mat31 obs_bias_gyro_,obs_bias_acc_;
    Vect<15> r_;
    Mat<15,15> W_;//information matrix (inverse of the prior covariance)
    Mat<15,15> J_;//Joint Jacobian
    SE3tc Tobs_;
    Mat101 xi_pose_;      // Ln(Tobs^{-1} T), 10-D (time component is 0 by construction)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FactorPriorInertial_HPP_ */
