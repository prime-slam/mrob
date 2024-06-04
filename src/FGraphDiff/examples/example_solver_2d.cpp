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


#include "../mrob/factor_graph_diff_solve.hpp"
#include "mrob/factor_graph.hpp"
#include "mrob/factors/factor1Pose2d_diff.hpp"
#include "mrob/factors/factor2Poses2d_diff.hpp"
#include "mrob/factors/nodePose2d.hpp"


#include <iostream>
# include <vector>

int main ()
{

    std::vector<mrob::factor_id_t> diff_factor_idx;

    // create a simple graph to solve:
    //     anchor ------ X1 ------- obs ---------- X2
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

    // Node 2, initialized at 0,0,0
    if (1)
    {
        std::shared_ptr<mrob::Node> n2(new mrob::NodePose2d(x));
        graph.add_node(n2);

        obs << -1 , -1 , 0;
        std::shared_ptr<mrob::DiffFactor> f2(new mrob::Factor2Poses2d_diff(obs,n2,n1,obsInformation));
        diff_factor_idx.emplace_back(graph.add_factor(f2));


        std::shared_ptr<mrob::Node> n3(new mrob::NodePose2d(x));
        graph.add_node(n3);

        obs << -1 , -1 , 0;
        std::shared_ptr<mrob::DiffFactor> f3(new mrob::Factor2Poses2d_diff(obs,n3,n2,obsInformation));
        diff_factor_idx.emplace_back(graph.add_factor(f3));
    }

    // solve the Gauss Newton optimization
    graph.print(true);
    for (int i = 0; i<10; i++)
        graph.solve(mrob::FGraphDiffSolve::GN);

    std::cout << "\nSolved, chi2 = " << graph.chi2() << std::endl;
    graph.print(true);


    std::cout << "==================================================\n" << std::endl;
    auto result = graph.get_estimated_state();
    for (auto x : result)
    {
        std::cout << x << std::endl;
    }

    // composing the gradient dr_dz for the problem
    auto A = graph.get_adjacency_matrix(); // has size |z| by |x|
    std::cout << "\nA = \n" << MatX(A) << std::endl;

    auto info = graph.get_information_matrix();
    std::cout << "\ninfo =\n" << MatX(info) << std::endl;

    auto b = graph.get_vector_b();
    std::cout << "\nb =\n" << MatX(b) << std::endl;

    auto W = graph.get_W_matrix();
    std::cout << "\nW =\n" << MatX(W) << std::endl;

    auto r = graph.get_vector_r();

    std::cout << "Residuals = " << r << std::endl;

    Eigen::SimplicialLDLT<mrob::SMatCol,Eigen::UpLoType::Lower, Eigen::AMDOrdering<mrob::SMatCol::StorageIndex>> alpha_solve;
    alpha_solve.compute(A.transpose()*W*A);
    SMatCol rhs(A.rows(),A.cols());
    rhs.setIdentity();
    std::cout << rhs << std::endl;

    MatX alpha = alpha_solve.solve(rhs); //
    std::cout << "\n alpha =\n" << alpha << std::endl;


    MatX errors_grads;
    errors_grads.resize(graph.get_dimension_state(), graph.get_dimension_obs());

    int f_index = 0;

    for (uint_t i = 0; i < diff_factor_idx.size(); ++i)
    {
        auto f = graph.get_factor(diff_factor_idx[i]);
        f->evaluate_jacobians();
        f->evaluate_residuals();
        f->evaluate_dr_dz();

        auto dr_dz = f->get_dr_dz();
        std::cout<< "\ndr_dz = " << dr_dz << std::endl;

        auto dr_dx = Mat3(f->get_jacobian().block(0,0,3,3));
        std::cout << "\ndr_dx = " << dr_dx << std::endl;

        auto Wf = f->get_information_matrix();
        std::cout << "\nW = " << Wf << std::endl;

        auto r = f->get_residual();
        std::cout << "\nresidual = " << r << std::endl;

        std::cout << dr_dx*Wf*dr_dz << std::endl;

        auto error = MatX(dr_dx*Wf*dr_dz);
        std::cout << "\nError value: " << error << std::endl; 

        errors_grads.block(f_index, f_index, f->get_dim_obs(), f->get_dim_obs()) <<  error;
        f_index += f->get_dim_obs();
    }

    errors_grads = -alpha*errors_grads;

    std::cout << "\nError_grads = \n" << errors_grads << std::endl;

    return 0;
}
