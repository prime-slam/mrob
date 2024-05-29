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
#ifndef MROB_FACTOR2POSES2D_DIFF_H
#define MROB_FACTOR2POSES2D_DIFF_H

#include "mrob/matrix_base.hpp"
#include "mrob/factor_diff.hpp"


namespace mrob{

    /**
     * Factor2Poses2d if a factor relating a 2 2dimensional poses
     * through a direct observation such that:
     * observation = h(nodeOrigin,nodeTarget) = x_target - x_origin
     * or
     * residual =  x_origin + observation - x_target
     *
     * With this arrangement, the linearized factor substracts the residual (r)
     * to the first order term of the nonlinear observation function:
     * || J dx - r ||
     *
     * This convention will be followed by all factors in this library, otherwise the optimization
     * will not work properly.
     *
     *
     * It is possible to update the target node if it has not been initialized.
     * By default is set to FALSE, so we must specify it to true in case we want
     * to update (for instance, for displacement odometry factors)
     *
     */
    class Factor2Poses2d_diff : public DiffFactor
    {
    public:
        Factor2Poses2d_diff(const Mat31 &observation, std::shared_ptr<Node> &nodeOrigin,
                       std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget=false,
                       DiffFactor::robustFactorType robust_type = DiffFactor::robustFactorType::QUADRATIC);
        ~Factor2Poses2d_diff() = default;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;
        void evaluate_chi2() override;
        void evaluate_dr_dz() override;
        void evaluate_d2r_dx_dz() override;
        MatRefConst get_dr_dz() const override;
        // std::vector<MatRefConst> get_d2r_dx_dz() override;

        MatRefConst get_obs() const override {return obs_;};
        VectRefConst get_residual() const override {return r_;};
        MatRefConst get_information_matrix() const override {return W_;};
        MatRefConst get_jacobian(mrob::factor_id_t /*id*/) const override {return J_;};
        void print() const override;

    protected:
        // The Jacobian's correspondent nodes are ordered on the vector<Node>
        // being [0]->J1 and [1]->J2
        // declared here but initialized on child classes
        Mat31 obs_, r_; //and residuals
        Mat3 dr_dz_; // derivative of residuals with reference to observations
        std::vector<Mat3> d2r_dx_dz_; // 2nd order derivative of residuals with reference to state and observation
        Mat3 W_;//inverse of observation covariance (information matrix)
        Mat<3,6> J_;//Joint Jacobian

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen
    };

    /**
     * Factor2Poses2dOdom_diff if a factor expressing the relation between consecutive
     * 2d nodes with odometry observations such that;
     *     residual = g (x_origin, observation) - x_dest
     *
     * Observation = [drot1, dtrans, drot2]
     *
     * We assume that this factor also updates the value of node destination
     * unless explicitly written
     *
     */
    class Factor2Poses2dOdom_diff : public Factor2Poses2d_diff
    {
    public:
        /**
         * Constructor of factor Odom. Conventions are:
         * 1) obs = [drot1, dtrans, drot2]
         * 2) This factor also updates the value of node destination according to obs
         */
        Factor2Poses2dOdom_diff(const Mat31 &observation, std::shared_ptr<Node> &nodeOrigin,
                           std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget=true,
                           DiffFactor::robustFactorType robust_type = DiffFactor::robustFactorType::QUADRATIC);
        ~Factor2Poses2dOdom_diff() = default;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;

    private:
        Mat31 get_odometry_prediction(Mat31 state, Mat31 motion);

    };

}

#endif //MROB_FACTOR2POSES2D_DIFF_H
