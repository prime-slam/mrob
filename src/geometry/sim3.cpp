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
 * sim3.cpp
 *
 *  Created on: July 7, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/sim3.hpp"


using namespace mrob;

Sim3::Sim3(const Mat4 &S):
    S_(S)
{
}

Sim3::Sim3(const Mat71 &nu) : S_(Mat4::Identity())
{
    this->Exp(nu);
}

Sim3::Sim3(const Sim3 &S) : S_(S.S())
{
}

const Eigen::Ref<const Mat4> Sim3::S() const
{
    return S_;
}


void Sim3::update_lhs(const Mat71 &dnu)
{

}


void Sim3::Exp()
{

}


std::string Sim3::toString() const
{
    std::stringstream ss;
    ss << this->S_;
    return ss.str();
}
