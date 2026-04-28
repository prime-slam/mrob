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
 * Factor1Landmark3d.hpp
 *
 *  Created on: Oct 31, 2025
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR1LANDMARK3D_HPP_
#define FACTOR1LANDMARK3D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The Factor1Poses3d is a vertex representing the distribution
 * of a nodePose3d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 *
 * In particular, the residual of this factor is:
 *   r = (x - obs) = l - l_obs
 */

class Factor1Landmark3d : public Factor
{
  public:
    Factor1Landmark3d(const Mat31 &observation, std::shared_ptr<Node> &n1,
        const Mat3 &obsInf, Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Landmark3d() = default;
    /**
     * Returns the chi2 error and fills the residual vector
     */
    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return obs_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};


  protected:
    Mat31 r_,obs_; //and residuals
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat3 J_;//Jacobian


};


}



#endif /* Factor1Landmard3d_HPP_ */
