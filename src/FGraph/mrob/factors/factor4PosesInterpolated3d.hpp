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
 * factor4PosesInterpolated3d.hpp
 *
 *  Created on: Dec 01, 2022
 *      Author: Kazii Botashev
 *              kazii.botashev@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR4POSESINTERPOLATED3D_HPP_
#define FACTOR4POSESINTERPOLATED3D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The Factor4PosesInterpolated3d is a vertex representing the distribution between
 * four nodePose3d, that is, it is expressing a Rigid Body Transformation
 * between two poses that are results of the interpolation from two intervals at timestamp tau in [0,1]. 
 * Each interval has a starting and ending pose that's why afterall we use 4 poses as nodes here.
 *
 * This factor might be used for the task in which we replace part of the nodes in factor graph
 * with interpolation reducing the number of nodes in optimization task, but keeping number of factors.
 * For that we decide to keep only for example each n-th node in factor graph and replace others with
 * interpolation. After that each interpolated factor is related interval between two nodes that
 * remain.
 *
 * The state is an observer RBT, and as we have commented, we need to specify
 * the four Nodes that the factor is connecting, which are provided by their
 * shared_ptr's.
 * We provide the node's Id to get the correspondent Jacobian
 * 
 * Having two pairs of nodes that represent start and end poses of interpolation interval
 * T_1_origin, T_1_target, T_2_origin, T_2_target, there are five possible cases of intersection of that
 * intervals:
 * 1) T_1_origin ... T_1_target ... T_2_origin ... T_2_target
 * 2) T_1_origin ... T_1_target = T_2_origin ... T_2_target
 * 3) T_1_origin = T_2_origin ... T_2_target = T_2_target 
 * 4) T_2_origin ... T_2_target = T_1_origin ... T_1_target
 * 5) T_2_origin = T_1_origin ... T_2_target = T_2_target 
 * This cases are defined via nodes_intersection_case parameter.
 *
 * The convention in the library r = f(x) - z.
 *
 * In this particular factor, we will follow a similar convention as in odometry 2d,
 * where we 'observe' the last pose, and thus, the relation between the transformation of poses is:
 *   T_o_ct(T_1_origin, T_1_target, tau_1) * T_obs = T_t_ct(T_2_origin, T_2_target, tau_2)
 *
 *
 *
 * T_o_ct is the transformation encoded by the 3D pose produced by interpolation at time tau_1 inside first interval
 * defined by nodes T_1_origin, T_1_target. 
 * T_t_ct target transformation that related to T_1_origin, T_1_target, tau_1.
 * T_obs observation transformation from pose obs (observed from interpolated T_o_ct)
 *
 * and the residual is thus:
 *   r = Ln ( T_o_ct * T_obs * T_t_ct^-1 )
 *
 * (equivalent to odometry 2d x_origin + observation - x_target)
 *
 *
 *
 *
 * The observations relate a pair of nodes. The order matters, since this will
 * affect the order on the Jacobian block matrix
 */

class Factor4PosesInterpolated3d : public Factor
{
  public:
    Factor4PosesInterpolated3d(const Mat4 &observation, 
            std::shared_ptr<Node> &nodeOriginFirstPair, std::shared_ptr<Node> &nodeTargetFirstPair, 
            std::shared_ptr<Node> &nodeOriginSecondPair, std::shared_ptr<Node> &nodeTargetSecondPair, 
            const float time, const float time_second, const Mat6 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    Factor4PosesInterpolated3d(const SE3 &observation, 
            std::shared_ptr<Node> &nodeOriginFirstPair, std::shared_ptr<Node> &nodeTargetFirstPair,
            std::shared_ptr<Node> &nodeOriginSecondPair, std::shared_ptr<Node> &nodeTargetSecondPair, 
            const float time_first, const float time_second, const Mat6 &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor4PosesInterpolated3d() override = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const;

    MatRefConst get_obs() const override {return Tobs_.T();}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian([[maybe_unused]]factor_id_t id = 0) const override {return J_;}

  protected:
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    // being [0]->J_origin and [1]->J_target
    // declared here but initialized on child classes
    SE3 Tobs_; // Transformation from observation. NOTE: In Xorigin frame
    Mat61 r_; //and residuals
    SE3 Tr_; // Residual Transformation
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat<6,24> J_;//Joint Jacobian
    float time_origin; //Time if interpolated pose which is origin in factor
    float time_target; //Time if interpolated pose which is target in factor
    int nodes_intersection_case;
    int id_a_0;
    int id_b_0;
    int id_a_1;
    int id_b_1;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR4POSESINTERPOLATED3D_HPP_ */