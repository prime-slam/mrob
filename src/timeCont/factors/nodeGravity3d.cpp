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

#include <iostream>
#include <cassert>

using namespace mrob;

NodeGravity3d::NodeGravity3d(const Mat31 &initial_x, Node::nodeMode mode) :
    Node(3, mode), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 3 && "NodeGravity3d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodeGravity3d:: Incorrect dimension on initial state cols" );
}


void NodeGravity3d::update(VectRefConst &dx)
{
    Mat31 dxf = dx;
    // remove non-tangent compoenent to the update as (I-gg) XXX this is NOT done in the factor
    state_ += dxf;
    regenerate();
}

void NodeGravity3d::regenerate(void)
{
    state_ = state_/state_.norm() * 9.8;
}

void NodeGravity3d::update_from_auxiliary(VectRefConst &dx)
{
    Mat31 dxf = dx;
    state_ = auxiliaryState_ + dxf;
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
        << ", state = \n" << state_;
}
