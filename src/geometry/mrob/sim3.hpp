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
 *  * S = [R t]
 *     [0 s]
 * 
 * Lie Coordinates:
 * nu = [theta, pho, lambda] = [xi, lambda]
 * 
 * For now it has the minimum functionalities to optimize
*/

namespace mrob{

class Sim3
{
public:
    Sim3(const Mat4 &S = Mat4::Identity());
    Sim3(const Mat71 &nu);
    Sim3(const Sim3 &S);
    /**
     * This is our *default* way to update transformations, from the Left hand side of T
     * Updates the current transformation with the incremental d\nu \in sim3
     * T'=Exp(\nu) * T
     */
    void update_lhs(const Mat71 &dnu);
    /**
     * Exponential Mapping
    */
    void Exp(const Mat4 &xi_hat);
    /**
     * T method returns a matrix 4x4 of the sim3 transformation. Ref<> is more convinient than
     * the matrix for the factor/nodes base class definitions and python bindings
     */
    const Eigen::Ref<const Mat4> S() const;

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

}// namespace

