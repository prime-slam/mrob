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
 * factor_diff.hpp
 *
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_DIFF_HPP_
#define FACTOR_DIFF_HPP_

#include <vector>
#include <memory>


#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"


namespace mrob{

class Factor;
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

    virtual MatRefConst get_dr_dz() const = 0;

    // virtual std::vector<MatRefConst> get_d2r_dx_dz() const = 0;
};

}

#endif /* FACTOR_DIFF_HPP_ */
