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
 * nodeSO3.cpp
 *
 *  Created on: Jul 18, 2025
 *      Author: Ivan Kakurin
 *              i.kakurin@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

 #include "mrob/factors/nodeSO3.hpp"

 #include <iostream>
 #include <cassert>
 
 using namespace mrob;
 
 NodeSO3::NodeSO3(const Mat3 &initial_x, Node::nodeMode mode) :
         Node(3,mode), state_(initial_x), auxiliaryState_(initial_x)
 {
     assert(isSO3(initial_x) && "NodeSO3:: Incorrect initial state, not an element of SO3" );
 }

NodeSO3::NodeSO3(const SO3 &initial_x, Node::nodeMode mode) :
		 Node(3, mode), state_(initial_x), auxiliaryState_(initial_x)
{
	assert(isSO3(initial_x.R()) && "NodeSO3:: Incorrect initial state, not an element of SO3" );
}
 
 
 
 void NodeSO3::update(VectRefConst &dx)
 {
     Mat31 dxf = dx;
 
     // Rx and x are always sync, i.e., Rx = exp(x^)
     state_.update_lhs(dxf);
     // XXX regeneration of state is required, for now we do it every time. random? count?
     // XXX is it necessary?
     state_.regenerate();
 }
 
 void NodeSO3::update_from_auxiliary(VectRefConst &dx)
 {
     Mat31 dxf = dx;
     state_ = auxiliaryState_;//we update from the auxiliary state
     state_.update_lhs(dxf);
 }
 
 void NodeSO3::set_state(MatRefConst &x)
 {
     // casting is necessary for SO3 constructor, it does not handle a ref TODO
     Mat3 newState = x;
     state_ = SO3(newState);
 }
 
 void NodeSO3::set_auxiliary_state(MatRefConst &x)
 {
     Mat3 newState = x;
     auxiliaryState_ = SO3(newState);
 }
 
 void NodeSO3::print() const
 {
     std::cout << "Printing NodeSO3: " << id_
         << ", state = \n" << state_.ln_vee() << ",\n SE3 matrix: \n";
     state_.print();
 }
 