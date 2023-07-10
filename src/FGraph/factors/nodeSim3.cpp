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
 * NodeSim3.cpp
 *
 *  Created on: July 10, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodeSim3.hpp"

#include <iostream>
#include <cassert>

using namespace mrob;

NodeSim3::NodeSim3(const Mat4 &initial_x, Node::nodeMode mode) :
        Node(7,mode), state_(initial_x), auxiliaryState_(initial_x)
{
    // TODO remove me
    //assert(initial_x.rows() == 6 && "NodeSim3:: Incorrect dimension on initial state rows" );
    //assert(initial_x.cols() == 1 && "NodeSim3:: Incorrect dimension on initial state cols" );
    assert(isSim3(initial_x) && "NodeSim3:: Incorrect initial state, not an element of Sim3" );
}

NodeSim3::NodeSim3(const Sim3 &initial_x, Node::nodeMode mode) :
		 Node(6, mode), state_(initial_x), auxiliaryState_(initial_x)
{
	assert(isSim3(initial_x.S()) && "NodeSim3:: Incorrect initial state, not an element of Sim3" );
}


void NodeSim3::update(VectRefConst &dx)
{
    Mat71 dxf = dx;

    // Tx and x are always sync, i.e., Tx = exp(x^)
    state_.update_lhs(dxf);
    // XXX regeneration of state is required, for now we do it every time. random? count?
    // XXX is it necessary?
    state_.regenerate();
}

void NodeSim3::update_from_auxiliary(VectRefConst &dx)
{
    Mat71 dxf = dx;
    state_ = auxiliaryState_;//we update from the auxiliary state
    state_.update_lhs(dxf);
}

void NodeSim3::set_state(MatRefConst &x)
{
	// casting is necessary for SE3 constructor, it does not handle a ref TODO
	Mat4 newState = x;
    state_ = Sim3(newState);
}

void NodeSim3::set_auxiliary_state(MatRefConst &x)
{
	Mat4 newState = x;
    auxiliaryState_ = Sim3(newState);
}

void NodeSim3::print() const
{
    std::cout << "Printing NodeSim3: " << id_ << "\n";
    state_.print();
}
