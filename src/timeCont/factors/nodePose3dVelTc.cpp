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
 * NodePose3dVelTc.cpp
 *
 *  Created on: Sept 13, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodePose3dVelTc.hpp"

#include <iostream>
#include <cassert>

using namespace mrob;

NodePose3dVelTc::NodePose3dVelTc(const Mat5 &initial_x, Node::nodeMode mode) :
        Node(9,mode), state_(initial_x), auxiliaryState_(initial_x)
{
}

NodePose3dVelTc::NodePose3dVelTc(const SE3tc &initial_x, Node::nodeMode mode) :
		 Node(9, mode), state_(initial_x), auxiliaryState_(initial_x)
{
}


void NodePose3dVelTc::update(VectRefConst &dx)
{
    Mat91 dxf = dx;

    // Tx  = Exp(xi,t=0) Tx
    SE3vel update = SE3vel(dxf);
    Mat5 res;
    res = update.T() * state_.T();
    state_ =  SE3tc(res);
    // XXX regeneration of state is required, for now we do it every time. random? count?
    //state_.regenerate();
}

void NodePose3dVelTc::update_from_auxiliary(VectRefConst &dx)
{
    Mat91 dxf = dx;
    state_ = auxiliaryState_;//we update from the auxiliary state
    SE3vel update = SE3vel(dxf);
    Mat5 res;
    res = update.T() * state_.T();
    state_ =  SE3tc(res);
}

void NodePose3dVelTc::set_state(MatRefConst &x)
{
	// casting is necessary for SE3 constructor, it does not handle a ref TODO
	Mat5 newState = x;
    state_ = SE3tc(newState);
}

void NodePose3dVelTc::set_auxiliary_state(MatRefConst &x)
{
	Mat5 newState = x;
    auxiliaryState_ = SE3tc(newState);
}

void NodePose3dVelTc::set_time_stamp(double t)
{
    time_stamp_ = t;
    state_.set_time(t);
    auxiliaryState_.set_time(t);
}

void NodePose3dVelTc::print() const
{
    std::cout << "Printing NodePose3dVelTc: " << id_
        << ", state = \n" << state_.Ln() << ",\n SE3tc matrix: \n";
    state_.print();
}
