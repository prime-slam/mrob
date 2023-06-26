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

#ifndef DIFFERENTIAL_FACTOR_HPP_
#define DIFFERENTIAL_FACTOR_HPP_


#include "mrob/factor.hpp"
#include "mrob/node.hpp"

namespace mrob{

class DiffFactor : public Factor
{

    DiffFactor(robustFactorType factor_type = QUADRATIC, uint_t potNumberNodes = 5);
    virtual ~DiffFactor() = default;
    /**
     * This is the only unique new method that requires to be passed, however it is key
     * for the correct functioning of the new generation 
     * 
     * TODO later use public virtual inheritance
     * 
     * Dimenstion are |obs| x |state|
    */
    virtual MatRefConst calculate_derivative_obs_state() = 0;
};

}//namespace

#endif /* FACTOR_HPP_ */