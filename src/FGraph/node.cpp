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
 * node.cpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/node.hpp"

using namespace mrob;

Node::Node(uint_t dim, nodeMode mode):
		 id_(0), dim_(dim), node_mode_(mode), isConnected2EF_(false)
{
}

Node::~Node()
{
}


// support function for 2D poses
double mrob::wrap_angle(double angle)
{
    double pi2 = 2 * M_PI;

    while (angle < -M_PI) angle += pi2;
    while (angle >= M_PI) angle -= pi2;

    return angle;
}
