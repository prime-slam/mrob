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
 * PCRegistration.hpp
 *
 *  Created on: Jan 22, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PC_REGISTRATION_HPP_
#define PC_REGISTRATION_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"

namespace mrob{
/**
 * This header provides the available functions for Point Cloud Registration using the
 * following methods. It assumes Data association and a correct index order for associated pairs.
 *
 * These set of methods calculates the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = T*xh and
 *  being xh and yh the homogeneous description of those points [x y z 1].
 */
namespace PCRegistration{

/**
 *  This solution is based on the paper by Arun et al. "Least-squares fitting of two 3-D point sets", 1987
 *  Given N points x = x_1,x_2,x_3 and their correspondences y_i, calculate the
 *  transformation T = [R t], such as: min sum (y_i - (R * x_i + t))^2
 *
 *  An improvement over it was proposed by Sinji Umeyama 1991, which uniqueness determine R
 *
 *  3 points in x and y are the minimum requirements, and provide the right solution IF
 *  there is no noise.
 *
 *  Returns 0 if failed and 1 if a correct solution was found
 */
int arun(MatRefConst X, MatRefConst Y, SE3 &T);




/**
 * Custom implementation of the GICP using SE3 optimization (improvement over Euler rotations)
 *
 * This method calculates the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = R*xh + t
 *
 *
 * T = min sum || y - Tx ||_S^2
 *
 * The covariances provided are of the form S = R diag(e,1,1) R', so they MUST have been already processed.
 * The right way is a block matrix of covariances of the form Cov = [Cov_1, Cov_2,..., Cov_N], i.e, Cov \in R^{3x3N}
 *
 * Returns the number of iterations until convergence
 */
int gicp(MatRefConst X, MatRefConst Y,
        MatRefConst covX, MatRefConst covY, SE3 &T, double tol = 1e-4);


/**
 * Weighted point Cloud registration using SE3 optimization
 *
 * This method calculates the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = R*xh + t
 *
 *
 * T = min sum w|| y - Tx ||^2
 *
 * The weights provided are of the form
 *
 * Returns the number of iterations until convergence
 */
int weighted_point(MatRefConst X, MatRefConst Y,
                   VectRefConst w, SE3 &T, double tol = 1e-4);


/**
 *  This solution is based on the paper by Arun et al. "Least-squares fitting of two 3-D point sets", 1987
 *  Given N points x = x_1,x_2,x_3 and their correspondences y_i, calculate the
 *  transformation S = [sR t], such as: min sum (y_i - (sR * x_i + t))^2
 *
 *  An improvement over it was proposed by Sinji Umeyama 1991, which uniqueness determine R
 *
 *  3 points in x and y are the minimum requirements, and provide the right solution IF
 *  there is no noise.
 *
 *  Returns 0 if failed and 1 if a correct solution was found
 */
int scaled_arun(MatRefConst X, MatRefConst Y, Mat4 &S);

}}//namespace
#endif /* PC_REGISTRATION_HPP_ */
