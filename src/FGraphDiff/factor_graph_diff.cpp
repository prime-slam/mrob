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
 * factor_graph.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <iostream>
#include <mrob/factor_graph_diff.hpp>


using namespace mrob;

FGraphDiff::FGraphDiff()
{}

FGraphDiff::~FGraphDiff()
{
    factors_.clear();
    nodes_.clear();
    eigen_factors_.clear();
}

factor_id_t FGraphDiff::add_factor(std::shared_ptr<DiffFactor> &factor)
{
    factor->set_id(factors_.size());
    factors_.emplace_back(factor);
    obsDim_ += factor->get_dim_obs();
    return factor->get_id();
}

factor_id_t FGraphDiff::add_eigen_factor(std::shared_ptr<DiffEigenFactor> &factor)
{
    factor->set_id(eigen_factors_.size());
    eigen_factors_.emplace_back(factor);
    return factor->get_id();
}
