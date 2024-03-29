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
 * time_profiling.hpp
 *
 *  Created on: Aug 14, 2019
 *      Author: Gonzalo Ferrer
 */

#ifndef TIME_PROFILING_HPP_
#define TIME_PROFILING_HPP_

#include <chrono>
#include <unordered_map>
#include <string>


namespace mrob {
using Ttim = std::chrono::microseconds;


/**
 * Class TimeProfiling creates a simple object that stores time
 * profiles for different functions and displays them.
 */


class TimeProfiling
{
public:
    /**
     * Constructor, no parameters reqauired
     */
    TimeProfiling();
    /**
     * Destructor, nothing to free
     */
    ~TimeProfiling();
    /**
     * Reset method
     */
    void reset();
    /**
     * start
     */
    void start();
    /**
     * stop() records given the string the time spent since last start() call
     */
    void stop(const std::string &key="");
    /**
     * print: displays the information gathered so far
     */
    void print();
    /**
     * total_time: returns the number of microsecond
     * accumulated in the class
     */
    double total_time();

protected:
    std::chrono::steady_clock::time_point t1_;
    std::unordered_map<std::string, double> time_profiles_;
};

}


#endif /* TIME_PROFILING_HPP_ */
