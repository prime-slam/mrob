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
 * filter_lidar_inertial.hpp
 *
 *  Created on: Feburary 1, 2024
 *      Author: Ahmed Baza
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef FILTERLIDARINERTIAL_HPP_
#define FILTERLIDARINERTIAL_HPP_


#include "mrob/matrix_base.hpp"


namespace mrob
{

class FilterLidarInertial
{
  public:
    // Here constructor shou,d have all covariances values and hyperparams.
    FilterLidarInertial();
    ~FilterLidarInertial();

    void read_observations();
    void solve();

};
}//namespace



#endif /* FILTERLIDARINERTIAL_HPP_ */