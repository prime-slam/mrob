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
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
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
    diff_factors_.clear();
}

factor_id_t FGraphDiff::add_factor(std::shared_ptr<DiffFactor> &factor)
{
    factor->set_id(diff_factors_.size());
    diff_factors_.emplace_back(factor);
    obsDim_ += factor->get_dim_obs();
    return factor->get_id();
}

std::shared_ptr<DiffFactor> &mrob::FGraphDiff::get_factor(factor_id_t key)
{
    assert(key < diff_factors_.size() && "FGraphDiff::get_factor: incorrect key");
    return diff_factors_[key];
}