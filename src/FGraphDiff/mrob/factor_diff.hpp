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
 * factor.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_DIFF_HPP_
#define FACTOR_DIFF_HPP_

#include <vector>
#include <memory>

#include "../FGraph/mrob/factor.hpp"


namespace mrob{

/**
 * DiffFactor class is a base pure abstract class defining factors,
 * the second type of vertexes on factor graphs (bipartite).
 * Factors keep track of all their neighbour nodes they are connected to.
 *
 * By convention, the residuals r_i are ALWAYS formulated as follows:
 *
 * -------------------------------------------
 * |           r(x) =  h(x) - z              |
 * -------------------------------------------
 *
 * otherwise the optimization will not work properly.
 */


class DiffFactor : public Factor
{
public:
    DiffFactor(uint_t dim, uint_t allNodesDim, robustFactorType factor_type = QUADRATIC, uint_t potNumberNodes = 5);
    virtual ~DiffFactor();
    /**
     * @brief evaluate derivative of residuals with reference to observations
     */
    virtual void evaluate_dr_dz() = 0;
    /**
     * @brief evaluate 2nd order derivative of residuals with reference to state and observation
     * 
     */
    virtual void evaluate_d2r_dx_dz() =0;
};

/**
 * Abstract class DiffEigenFactor. This is a factor with extra methods than DiffFactor
 * which requires a new base abstract class.
 *
 * The Eigen factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the current state, which is a geometric entity
 * estimated a priory on each iteration, e.g. a plane. The resultant topology
 * is N nodes connecting to the eigen factor.
 *
 * Hence, the new method get state, for instance a plane, but this state is outside the FGraphDiff optimization, that
 * is why we can consider this approach non-parametric
 *  - get_state()
 *
 * NOTE: due to its nature, multiple observation can be added to the same EF,
 * meaning we need to create a constructor PLUS an additional method
 *  - add_point()
 *
 * In order to build the problem we would follow the interface specifications by FGraphDiff
 * but we need extra methods and variables to keep track of the neighbours. For instance, we need
 * to get Jacobians and Hessian matrices
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 * XXX should we store all points?
 */
class DiffEigenFactor : public EigenFactor
{
public:
    DiffEigenFactor(robustFactorType factor_type = QUADRATIC, uint_t potNumberNodes = 5);
    virtual ~DiffEigenFactor() = default;
};

}

#endif /* FACTOR_DIFF_HPP_ */
