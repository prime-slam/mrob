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
 * factor_graph_diff.hpp
 *
 *  Created on: May 8, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef SRC_FACTOR_GRAPH_DIFF_HPP_
#define SRC_FACTOR_GRAPH_DIFF_HPP_

#include "mrob/factor_graph_solve.hpp"
#include <vector>

namespace mrob{


/**
 * Class Factor Graph Differential
 * 
 * This class is just an extension from FGraph to return the correct
*/
class FGraphDiff: public FGraphSolve
{
public:
    FGraphDiff();
    /**
     * Creates virtual destructor
    */
    virtual ~FGraphDiff();

    /**
     * This functions returns a list of derivatives wrt to the observation
     * The index corresponds to the factor id 
    */
    std::vector<Mat1X> get_diff_obs();

    /**
     * Add diff factor, same as in FGraph.
    */
   factor_id_t add_diff_factor(std::shared_ptr<DiffFactor> &diff_factor);
   /**
     * Overwrite to not add by mistake any factor that is not of the right type
   */
   factor_id_t add_factor(std::shared_ptr<Factor> &factor);
   
private:

}


#endif /* SRC_FACTOR_GRAPH_DIFF_HPP_ */