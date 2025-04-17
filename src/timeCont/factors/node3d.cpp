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
 * nodePose3dVelTc.hpp
 *
 *  Created on: April 17, 2025
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/node3d.hpp"


#include <iostream>

using namespace mrob;



Node3d::Node3d(const Mat31 &initial_x, Node::nodeMode mode):
    Node(3,mode), state_(initial_x), auxiliaryState_(initial_x)
{
}

void Node3d::update(VectRefConst &dx)
{
    state_ =  state_ + dx;
}

void Node3d::update_from_auxiliary(VectRefConst &dx)
{
    state_ = auxiliaryState_;//we update from the auxiliary state
    state_ =  state_ + dx;
}

void Node3d::set_state(MatRefConst &x)
{
	// casting is necessary for SE3 constructor, it does not handle a ref TODO
    state_ = x;
}

void Node3d::set_auxiliary_state(MatRefConst &x)
{
    auxiliaryState_ = x;
}


void Node3d::print() const
{
    std::cout << "Printing Node3d: " << id_
        << ", state = \n" << state_ << "\n ";
}
