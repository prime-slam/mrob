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
 * example_solver_2d.cpp
 *
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/factor_graph_diff_solve.hpp"
#include "mrob/factor_graph.hpp"
#include "mrob/factors/factor1Pose2d_diff.hpp"
#include "mrob/factors/factor2Poses2d_diff.hpp"
#include "mrob/factors/nodePose2d.hpp"


#include <iostream>
# include <vector>

int main ()
{

    std::vector<mrob::factor_id_t> diff_factor_idx;

    mrob::FGraphDiffSolve graph(mrob::FGraphDiffSolve::ADJ);

    // Initial node is defined at 0,0,0, and anchor factor actually observing it at 0
    mrob::Mat31 x, obs;
    x = mrob::Mat31::Random()*0.1;
    obs = mrob::Mat31::Zero();
    // Nodes and factors are added to the graph using polymorphism. That is why
    // we need to declare here what specific kind of nodes or factors we use
    // while the definition is an abstract class (Node or DiffFactor)
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose2d(x));
    graph.add_node(n1);

    Mat3 obsInformation= Mat3::Identity();
    std::shared_ptr<mrob::DiffFactor> f1(new mrob::Factor1Pose2d_diff(obs,n1,obsInformation*1e6));
    diff_factor_idx.emplace_back(graph.add_factor(f1));

    std::shared_ptr<mrob::Node> n2(new mrob::NodePose2d(x));
    graph.add_node(n2);

    obs << -1 , -1 , 0;
    std::shared_ptr<mrob::DiffFactor> f2(new mrob::Factor2Poses2d_diff(obs,n2,n1,obsInformation));
    diff_factor_idx.emplace_back(graph.add_factor(f2));

    obs << 1, 1, 0;
    std::shared_ptr<mrob::DiffFactor> gnss_2(new mrob::Factor1Pose2d_diff(obs,n2, obsInformation*1e4));
    diff_factor_idx.emplace_back(graph.add_factor(gnss_2));

    std::shared_ptr<mrob::Node> n3(new mrob::NodePose2d(x));
    graph.add_node(n3);

    obs << -1 , -1 , 0;
    std::shared_ptr<mrob::DiffFactor> f3(new mrob::Factor2Poses2d_diff(obs,n3,n2,obsInformation));
    diff_factor_idx.emplace_back(graph.add_factor(f3));

    obs << 2, 2, 0;
    std::shared_ptr<mrob::DiffFactor> gnss_3(new mrob::Factor1Pose2d_diff(obs,n3,obsInformation*1e4));
    diff_factor_idx.emplace_back(graph.add_factor(gnss_3));

    graph.build_jacobians();

    std::cout << "\nA = \n" << MatX(graph.get_dx_dz()) << std::endl;



    return 0;
}
