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
 * FactorGravityAlign.hpp
 *
 *  Created on: Jul 18, 2025
 *      Author: Ivan Kakurin
 *              i.kakurin@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


 #ifndef FACTORGRAVITYALIGN_HPP_
 #define FACTORGRAVITYALIGN_HPP_
 
 
 #include "mrob/matrix_base.hpp"
 #include "mrob/SO3.hpp" //requires including and linking SE3 library
 #include "mrob/factor.hpp"
 
 namespace mrob{
 
 /**
  * The FactorGravityAlign is a vertex representing the distribution
  * of a nodeSO3, pretty much like an anchoring factor.
  *
  * The state is an observed orientation, coincident with the node state it is connected to.
  *
  * In particular, the residual of this factor is:
  *   r = (x - obs) = Rx * Robs^{-1}
  */
 
 class FactorGravityAlign : public Factor
 {
  public:
   FactorGravityAlign(const Mat31 &acc, const Mat31 &grav, std::shared_ptr<Node> &n1, const Mat3 &obsInf,
             Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
             
   FactorGravityAlign(const Mat31 &acc, std::shared_ptr<Node> &n1, const Mat3 &obsInf,
             Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);

    ~FactorGravityAlign() = default;
    /**
     * Returns the chi2 error and fills the residual vector
     */
    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return a_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};
 
 
  protected:
    Mat31 a_;
    Mat31 g_;
    Mat31 Ra_;
    Mat31 r_; //and residuals
    SO3 Robs_, Rr_;//Transformation for the observation and the residual
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat3 J_;//Jacobian
 
 
 };
 
 
 }
 
 
 
 #endif /* FACTORGRAVITYALIGN_HPP_ */
 