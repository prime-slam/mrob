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
 * EigenFactorPlane.hpp
 *
 *  Created on: Oct 26 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef PIFACTORPLANE_HPP_
#define PIFACTORPLABE_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"


namespace mrob{

/**
 * Reimplementation of pi-factor from Zhou et al ICRA 2021
 * This factor is mostly based on the EF, using the squared root form of the S matrix
 * and connects two nodes, the pose and the plane landmark
 *
 * sum ||pi' T p_i||^2 = || \sqrt(S)' T' pi||^2
 *
 * the two nodes connected at T (pose) and pi (plane 4d)
 * The observation is the sqrt(S), which in clude all points observed
 */
class PiFactorPlane: public Factor{
public:
    /**
     * Creates the Pi factor.
     *
     */
    PiFactorPlane(const Mat4 &Sobservation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodePlane,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~PiFactorPlane() = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual void evaluate_residuals() override;
    /**
     * Evaluates the Jacobians
     */
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const override;

    MatRefConst get_obs() const override {return Sobs_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id*/) const override {return J_;};



private:
    Mat41 r_; //residuals
    Mat<4,10> J_;//Jacobians dimensions obs x [plane(4) + pose(6)]
    Mat4 W_;//inverse of observation covariance (information matrix)
    bool reversedNodeOrder_;
    // intermediate variables to keep
    Mat41 plane_;
    Mat4 Sobs_, S_mul_T_transp_;// we store this bariable for later calcuating the Jacobian faster

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}
#endif /* PiFactorPlane_HPP_ */
