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
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef MROB_FACTOR1POSE2D_DIFF_H
#define MROB_FACTOR1POSE2D_DIFF_H

#include "mrob/matrix_base.hpp"
#include "mrob/factor_diff.hpp"

using namespace mrob;


/**
 * The Factor1Poses2d_diff is a vertex representing the distribution
 * of a nodePose2d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 *
 * In particular, the residual of this factor is:
 *   r = x - obs
 */


namespace mrob{
    class Factor1Pose2d_diff : public DiffFactor
    {
    public:
        Factor1Pose2d_diff(const Mat31 &observation, std::shared_ptr<Node> &n1,
                const Mat3 &obsInf, DiffFactor::robustFactorType robust_type = DiffFactor::robustFactorType::QUADRATIC);
        ~Factor1Pose2d_diff() = default;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;
        void evaluate_chi2() override;
        void evaluate_dr_dz() override;
        // void evaluate_d2r_dx_dz() override;
        MatRefConst get_dr_dz() const override;
        // std::vector<MatRefConst> get_d2r_dx_dz() const override;

        void print() const override;

        MatRefConst get_obs() const override {return obs_;};
        VectRefConst get_residual() const override {return r_;};
        MatRefConst get_information_matrix() const override {return W_;};
        MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};
        

    protected:
        Mat31 obs_, r_; //and residuals
        Mat3 dr_dz_; // derivative of residuals with reference to observations
        std::vector<Mat3> d2r_dx_dz_; // 2nd order derivative of residuals with reference to state and observation
        Mat3 W_;//inverse of observation covariance (information matrix)
        Mat3 J_;//Jacobian
    };
}

#endif //MROB_FACTOR1POSE2D_DIFF_H
