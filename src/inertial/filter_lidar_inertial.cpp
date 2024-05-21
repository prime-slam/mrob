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
 * filter_lidar_inertial.cpp
 *
 *  Created on: Feburary 1, 2024
 *      Author: Ahmed Baza
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include "mrob/filter_lidar_inertial.hpp"


using namespace mrob;
FilterLidarInertial::FilterLidarInertial()
{

}

FilterLidarInertial::~FilterLidarInerial()
{

}

void FilterLidarInertial::read_observations(const InertialData *LaodingData)
{
  this-> Data = LaodingData;
  // fill in the function and change the heather.
}
void print_data(){

}; 
void propagate(){

}
void solve(){

}