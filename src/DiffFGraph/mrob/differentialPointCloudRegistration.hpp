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
 * factor_graph_solve.hpp
 *
 *  Created on: June 5, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef DIFFERENTIAL_POINT_CLOUD_REGISTRATION_
#define DIFFERENTIAL_POINT_CLOUD_REGISTRATION_

#include "mrob/differential_factor_graph.hpp"

/**
 * This class creates a specific problem for PC registration and allows a simple function to
 * obtained the backpropagated gradients after finding a solution.
 * 
 * The problem is described as the optimization of the following expression:
 * 
 * $$min_{T} \sum_n^N = ||T x_n - y_n||_{w_n}^2$$
 * 
 * The gradients are obtained by the Implicit Theorem, assuming we hav ereached a stationary
 * solution to the optimization problem.
 * 
 * Input:
 *  - X: Point cloud of points, dim Nx3
 *  - Y: Point cloud of points, dim Nx3
 *  - W: weights for each pair of PC. Nx1 (vector)
 * 
*/

class DifferentialPCRegistration{
public:
    DifferentialPCRegistration(MatRefConst X, MatRefConst Y, VectRefConst w);
    virtual ~DifferentialPCRegistration();
    MatRefConst get_differential_observations(MatRefConst ground_truth)

};



#endif