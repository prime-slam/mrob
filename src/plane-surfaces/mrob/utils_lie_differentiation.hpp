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
 * utils_lie_differentiation.hpp
 *
 *  Created on: August 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef UTILS_LIE_DIFFERENTIATION_HPP_
#define UTILS_LIE_DIFFERENTIATION_HPP_


#include "mrob/matrix_base.hpp"



namespace mrob{

/**
 * Utils for fast calcuations of gradients and hessian of Lie Algebra coordinates
 *
 */

/**
 * Returns the expression : J.col(i) = dQ/dx_i * pi , \forall i = 1,..,6
 * where the inputs are the Q matrix (symetric), the plane vector pi
 * and the coordinate of the derivative
*/
Mat<4,6> gradient_Q_x_pi(const Mat4 Q, const Mat41 pi);


/**
 * Returns the expression: H_l(i,j) =  pi' * d^2Q/dx_j*dx_i * pi
 * where the inputs are the Q matrix (symetric), the plane vector pi
 * and the coordinate of the derivative
 * 
 * The result is aggregated into a matrix (i,j) (it is symetric)
 * stored in the upper triangular view.
*/
Mat6 pi_t_x_hessian_Q_x_pi(const Mat4 Q, const Mat41 pi);

/**
 * Returns the expression : J.row(i) = pi' * G_i
 * 
*/
Mat<6,4> pi_t_times_lie_generatives(const Mat41 pi);

}


#endif /* UTILS_LIE_DIFFERENTIATION_HPP_ */
