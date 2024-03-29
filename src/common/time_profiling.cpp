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
 * time_profiling.cpp
 *
 *  Created on: Aug 14, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#include "mrob/time_profiling.hpp"
#include <iostream>

using namespace mrob;

TimeProfiling::TimeProfiling()
{

}

TimeProfiling::~TimeProfiling()
{

}

void TimeProfiling::reset()
{
    time_profiles_.clear();
    t1_ = std::chrono::steady_clock::now();
}

void TimeProfiling::start()
{
    t1_ = std::chrono::steady_clock::now();
}

void TimeProfiling::stop(const std::string &key)
{
    auto t2 = std::chrono::steady_clock::now();
    auto dif = std::chrono::duration_cast<Ttim>(t2 - t1_);
    if (time_profiles_.count(key))
        time_profiles_.at(key) += dif.count();
    else
        time_profiles_.emplace(key,  dif.count());
}


void TimeProfiling::print()
{
    double sum = total_time();

    std::cout << "\nTime profile for " << sum/1e3 << " [ms]: \n";
    for (auto &&t : time_profiles_)
        std::cout << t.first << " = " << t.second/sum *100 << "%,\n";
    std::cout << "\n";
}

double TimeProfiling::total_time()
{
    double sum = 0;
    for (auto &&t : time_profiles_)
        sum += t.second;
    return sum;
}
