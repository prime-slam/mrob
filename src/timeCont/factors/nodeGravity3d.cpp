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
 * NodeGravity3d.cpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodeGravity3d.hpp"
#include "mrob/SO3.hpp"

#include <Eigen/Geometry>   // cross product
#include <cassert>
#include <cmath>
#include <iostream>

using namespace mrob;

Mat<3,2> mrob::gravity_tangent_basis(const Mat31 &g)
{
    const Mat31 n = g.normalized();
    // Reference axis: the coordinate axis least aligned with g (well conditioned cross product)
    Mat31 axis = Mat31::Zero();
    Eigen::Index k;
    n.cwiseAbs().minCoeff(&k);
    axis(k) = 1.0;
    const Mat31 b1 = n.cross(axis).normalized();
    const Mat31 b2 = n.cross(b1);   // unit, since n and b1 are orthonormal. [b1, b2, n] is right handed.
    Mat<3,2> B;
    B.col(0) = b1;
    B.col(1) = b2;
    return B;
}

NodeGravity3d::NodeGravity3d(const Mat31 &initial_x, Node::nodeMode mode) :
    Node(2, mode), state_(initial_x), auxiliaryState_(initial_x), magnitude_(initial_x.norm())
{
    assert(magnitude_ > 0.0 && "NodeGravity3d:: initial gravity must be a non-zero vector");
}

Mat31 NodeGravity3d::retract(const Mat31 &g, const Mat21 &dx, matData_t magnitude)
{
    const Mat<3,2> B = gravity_tangent_basis(g);
    const Mat31 w = B * dx;                 // rotation vector, perpendicular to g
    const matData_t theta2 = w.squaredNorm();
    const Mat3 K = hat3(w);
    matData_t a, b;                          // Rodrigues: R = I + a K + b K^2
    if (theta2 < 1e-10)
    {
        a = 1.0 - theta2 / 6.0;
        b = 0.5 - theta2 / 24.0;
    }
    else
    {
        const matData_t theta = std::sqrt(theta2);
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / theta2;
    }
    const Mat31 g_new = g + a * (K * g) + b * (K * (K * g));
    return g_new * (magnitude / g_new.norm());   // removes numerical drift only
}

void NodeGravity3d::update(VectRefConst &dx)
{
    assert(dx.size() == 2 && "NodeGravity3d::update: the update has 2 DoF");
    const Mat21 dxf = dx;
    state_ = retract(state_, dxf, magnitude_);
}

void NodeGravity3d::update_from_auxiliary(VectRefConst &dx)
{
    assert(dx.size() == 2 && "NodeGravity3d::update_from_auxiliary: the update has 2 DoF");
    const Mat21 dxf = dx;
    state_ = retract(auxiliaryState_, dxf, magnitude_);
}

void NodeGravity3d::set_state(MatRefConst &x)
{
    // cast is done by Eigen
    state_ = x;
}

void NodeGravity3d::set_auxiliary_state(MatRefConst &x)
{
    auxiliaryState_ = x;
}

void NodeGravity3d::print() const
{
    std::cout << "Printing NodeGravity3d: " << id_
        << ", state = \n" << state_ << std::endl;
}
