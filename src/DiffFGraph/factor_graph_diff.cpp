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
 * factor_graph_diff.cpp
 *
 *  Created on: May 8, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factor_graph_diff.hpp"


using namespace mrob;

FGraphDiff::FGraphDiff(): FGraphSolve(FGraphSolve::ADJ)
{

}


factor_id_t FGraphDiff::add_factor(std::shared_ptr<DiffFactor> &factor)
{
    // same logic as in standard
    return 0;
}

// TODO need id ordering for the observations. Is this the factor id?