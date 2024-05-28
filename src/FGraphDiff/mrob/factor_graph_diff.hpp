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
 * factor_graph.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_GRAPH_DIFF_HPP_
#define FACTOR_GRAPH_DIFF_HPP_

//#include <unordered_map>
#include <deque>// for long allocations

#include "mrob/factor_diff.hpp"
#include "../FGraph/mrob/factor_graph.hpp"

namespace mrob{
/**
 * This class provides the general structure for encoding Factor Graphs and
 * to support the implementation of the inference solution to the joint probability P(x,u,z).
 * The solution to this joint probability is equivalent to a Nonlinear Least Squares (NLSQ) problem.
 *
 * Factor Graphs are bipartite graphs, meaning that we express the relations from a set of vertices "nodes"
 * which include our state variables through a set of vertices "factors", capturing the inherent distribution
 * of the nodes variables due to observations.
 * Bipartite is in the sense that edges of the graph are always from nodes to factors or vice versa.
 *
 * We require two abstract classes,
 *  - Class Node
 *  - Class Factor. here see factor.hpp for the conventions on residuals, observations, etc.
 *
 * XXX, actually key as addresses won't work in python interface. Better use id_t (uint)
 * Both data containers are stored in vectors (XXX prev unordered sets) whose keys are their addresses. By doing this, we can
 * iterate and quickly find elements in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data, such as information matrix, factorizations, etc.
 *
 */

class FGraphDiff : public FGraph{
public:
    FGraphDiff();
    virtual ~FGraphDiff();
    /**
     * Adds a factor, if it is not already on the set.
     * Note that the connecting nodes of the factor should be already
     * specified when creating the factor.
     *
     * This function includes the factor into its connected
     * nodes.
     *
     * Modifications of the structure of the graph are allowed
     * by removing the factor and adding the new updated one.
     *
     * returns factor id
     */
    factor_id_t add_factor(std::shared_ptr<DiffFactor> &factor);
    /**
     * Adds an Eigen Factor, the special factor that is not formulated
     * as a sum of residuals, but directly as a real value (eigenvalue)
     * and therefore it requires a different processing, apart from the
     * standard residual factors from above.
     */
    factor_id_t add_eigen_factor(std::shared_ptr<DiffEigenFactor> &factor);
    /**
     * get_node returns the node given the node id key, now a position on the data structure
     */
    std::shared_ptr<DiffFactor>& get_factor(factor_id_t key);
};



}

#endif /* FACTOR_Graph_DIFF_HPP_ */
