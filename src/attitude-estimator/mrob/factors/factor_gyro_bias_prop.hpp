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
 * FactorGyroBiasProp.hpp
 *
 *  Created on: Oct 25, 2025
 *      Author: Ivan Kakurin
 *              i.kakurin@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#ifndef FACTORGYROBIASPROP_HPP_
#define FACTORGYROBIASPROP_HPP_
 
 
#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"
#include "mrob/SE3.hpp"
#include "mrob/factor.hpp"
 
namespace mrob{

/**
* The FactorGyroBiasProp is a vertex representing the distribution
* of a nodeSO3, pretty much like an anchoring factor.
*
* The state is an observed orientation, coincident with the node state it is connected to.
*
* In particular, the residual of this factor is:
*   r = (x - obs) = Ro * Robs((w-b)dt) * Rtarget^{-1}
*/

class FactorGyroBiasProp : public Factor
{
  public:
    FactorGyroBiasProp(const Mat31 &gyroscope, const double &dt,
            std::shared_ptr<Node> &nodeOrigin, 
            std::shared_ptr<Node> &nodeTarget,
            std::shared_ptr<Node> &nodeBias, //a R^3 vector, ie nodeLandmark3d
            const Mat3 &obsInf, bool updateNodeTarget, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);

    ~FactorGyroBiasProp() = default;
    /**
     * Returns the chi2 error and fills the residual vector
     */
    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return Robs_.ln_vee();};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};
 
 
  protected:
    double dt_;
    Mat31 bias_, gyro_;
    Mat31 r_; //and residuals
    SO3 Robs_, Rr_;//Transformation for the observation and the residual
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat<3,9> J_;//Jacobian
    
    std::vector<uint_t> original_to_ordered_index_, jacobian_node_index_;//data structures to handle any order of nodeIds
 
 
};
 
 
}
 
 
 
#endif /* FACTORGYROBIASPROP_HPP_ */
 
