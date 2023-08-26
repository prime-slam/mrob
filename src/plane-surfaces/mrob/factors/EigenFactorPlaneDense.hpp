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
 * EigenFactorPlaneDense.hpp
 *
 *  Created on: Aug 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef EIGENFACTORPLANEDENSE_HPP_
#define EIGENFACTORPLANEDENSE_HPP_


#include "mrob/factors/EigenFactorPlaneBase.hpp"
#include "mrob/utils_lie_differentiation.hpp"


namespace mrob{

/**
 * Eigen factor Plane is a vertex that complies with the Fgraph standards
 * and inherits from base EigenFactor.hpp
 *
 * The Plane factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the plane, so the resultant topology
 * is N nodes connecting to the plane factor.
 *
 * NOTE: due to its nature, multiple observation can be added to the same EF,
 * meaning we need to create a constructor PLUS an additional method
 *  - add_point()
 *
 * This result is DENSE from the fact that the  hessian matrix is dense in
 * the sense that cross terms are not zero,
 * whereas the alternating EF obtained a block diagonal hessian.
 * 
 * In order to build the problem we would follow the interface specifications by FGraph
 * but we need extra methods and variables to keep track of the neighbours
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 */
class EigenFactorPlaneDense: public EigenFactorPlaneBase{
public:
    /**
     * Creates an Eigen Factor plane. The minimum requirements are 1 pose, which is not required
     * at this stage, but will be introduced when we add points/Q matrix.
     */
    EigenFactorPlaneDense(Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPlaneDense() = default;
    /**
     * Jacobians are not evaluated, just the residuals.
     * This function is calculating the current plane estimation
     */
    void evaluate_residuals();
    /**
     * Evaluates Jacobians, given the residual evaluated
     * It also evaluated the Hessians
     */
    void evaluate_jacobians();
    /**
     * Chi2 is a scaling of the plane error
     */
    void evaluate_chi2();


    /**
     * get jacobian returns the jacobian corresponding to the given node id.
     * @return
     */
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const;
    /**
     * get hessian returns the Hassian corresponding to the given node id.
     * @return
     */
    MatRefConst get_hessian(mrob::factor_id_t id = 0) const;

    /**
     * get hessian returns the Hassian corresponding to the given node id.
     * @return
     */
    MatRefConst get_hessian_block(mrob::factor_id_t id = 0, mrob::factor_id_t id2 = 0) const;



};

}
#endif /* EigenFactorPlaneDense_HPP_ */
