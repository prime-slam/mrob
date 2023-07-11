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
 * sim3.hpp
 *
 *  Created on: July 7, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SIM3_HPP_
#define SIM3_HPP_


#include "mrob/matrix_base.hpp"



/**
 * The sim(3) group
 *  * S = [R    t]
 *        [0 s^-1]
 * 
 * Lie Coordinates:
 * nu = [theta, pho, lambda] = [xi, lambda]
 * 
 * For now it has the minimum functionalities to optimize:
 *  - Constructor
 *  - Exponent (as a class method that updates its own state)
 *  - update lhs
 *  - transform (homogeneous) vector into scaled
*/

namespace mrob{

class Sim3
{
public:
    Sim3(const Mat4 &S = Mat4::Identity());
    Sim3(const Mat71 &nu);
    Sim3(const Sim3 &S);
    Sim3& operator=(const Sim3&) = default;
    Sim3& operator=(Sim3&&) = default;
    /**
     * This is our *default* way to update transformations, from the Left hand side of T
     * Updates the current transformation with the incremental d\nu \in sim3
     * T'=Exp(\nu) * T
     */
    void update_lhs(const Mat71 &dnu);
    /**
     * Exponential Mapping: From lie coordionates to sim3
    */
    void Exp(const Mat71 &nu);
    /**
     * T method returns a matrix 4x4 of the sim3 transformation. Ref<> is more convinient than
     * the matrix for the factor/nodes base class definitions and python bindings
     */
    const Eigen::Ref<const Mat4> S() const;
    /**
     * R method returns a matrix 3x3 of the SO3 rotation corresponding to the subblock matrix
     */
    Mat3 R() const;
    /**
     * t method returns translation
     */
    Mat31 t() const;
    /**
     * Get scale s, from matrix S.
     * Note that in the sim3, scale is inverted
    */
    matData_t scale() const;

    /**
     * Transform (homogeneous) vector as if SE3, ignoring scale
     * 
     *      S *[p] = [R*p + t]
     * 
    */
    Mat31 transform(const Mat31 &p) const;

    /**
     * Transform scaled gets a vector (homog) into scaled:
     * 
     *      S *[p] = [R*p + t] ~ [s(R*p + t)]
     *         [1]   [  s^-1 ]   [     1    ]
     * 
     * This is the same vector, but scaled and the equality holds since
     * the vector is in the projective group P^3 
    */
    Mat31 transform_scaled(const Mat31 &p) const;

    /**
     * Regenerate, does the following operation, only onm the SO3 part
     * S = [Exp(Ln(R)) t]
     *   = [0       s^-1]
     */
    void regenerate();

    void print(void) const;

    /**
     * @brief Generates string representation of the object
     *
     * @return std::string object to print
     */
    std::string toString() const;
protected:
    Mat4 S_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


bool isSim3(const Mat4 &S);


}// namespace
#endif /* SIM3_HPP_ */
