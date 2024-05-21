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
 * initializationFactorGyro.cpp
 *
 *  Created on: January 11, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/nodeInertial3d.hpp"
#include <iostream>

using namespace mrob;

NodeInertial3d::NodeInertial3d(const Mat<3,7> state,
                Node::nodeMode mode):
            Node(15,mode), state_(state), auxiliary_state_(state)
{
    //assert
    gyro_bias_ = state.col(5);
    acc_bias_ = state.col(6);
}

void NodeInertial3d::update(VectRefConst &dx)
{
    Vect<15> dxf = dx;
}


void NodeInertial3d::update_from_auxiliary(VectRefConst &dx)
{
}


void NodeInertial3d::set_state(MatRefConst &x)
{
    Mat<3,7> new_state = x;
    state_ = new_state;
    // Needs to update intermediate vars we are saving
}

void NodeInertial3d::set_auxiliary_state(MatRefConst &x)
{
}

void NodeInertial3d::print() const
{
    std::cout << "Printing Node Inertial 3D: " << id_
              << "\n state= \n" << state_
              << std::endl;
}

