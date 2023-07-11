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
 * scaled_alignment.cpp
 *
 *  Created on: July 11, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <memory>
#include <iostream>
#include "mrob/pc_registration.hpp"
#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/factor1Sim3Point2Point.hpp"
#include "mrob/factors/nodeSim3.hpp"

using namespace mrob;
//using namespace Eigen;

int PCRegistration::scaled_alignment(MatRefConst X, MatRefConst Y, Mat4 &S)
{
    assert(X.cols() == 3  && "PCRegistration::scaled_alignment: Incorrect sizing, we expect Nx3");
    assert(X.rows() >= 3  && "PCRegistration::scaled_alignment: Incorrect sizing, we expect at least 3 correspondences (not aligned)");
    assert(Y.rows() == X.rows()  && "PCRegistration::scaled_alignment: Same number of correspondences");
    uint_t N = X.rows();

    // We will use Fgraph routine to create the following graph:
    // - 1 node sim3. this is the varaible we want to solve
    // - N factors as observation. Minimum 3 obs
    mrob::FGraphSolve graph(mrob::FGraphSolve::ADJ);
    std::shared_ptr<mrob::Node> n0(new mrob::NodeSim3(mrob::Sim3()));
    graph.add_node(n0);

    // Add observations, as provided in the function
    Mat3 obsInf = Mat3::Identity();
    Mat31 x,y;
    for (unsigned int i = 0; i < N; i++)
    {
        x = X.row(i);
        y = Y.row(i);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Sim3Point2Point(x,y,n0,obsInf));
        auto id_f = graph.add_factor(f);
        std::cout << "adding factor " << id_f << std::endl;
    }
    
    // Solve the graph
    graph.solve();
    Mat4 solution;
    solution = graph.get_estimated_state()[0];

    S = solution;

    return 1;
}
