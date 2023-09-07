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
 * EigenFactorPlaneBase.cpp
 *
 *  Created on: Aug 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneBase.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"

using namespace mrob;

EigenFactorPlaneBase::EigenFactorPlaneBase(Factor::robustFactorType robust_type):
        EigenFactor(robust_type),
        planeEstimation_{Mat41::Zero()},
        numberPoints_{0}
{
}

void EigenFactorPlaneBase::add_point(const Mat31& p, std::shared_ptr<Node> &node, matData_t &W)
{
    // Pose has been observed, data has been initialized and we simply add point
    auto id = node->get_id();
    if (reverseNodeIds_.count(id) > 0)
    {
        uint_t localId = reverseNodeIds_[id];
        allPlanePoints_.at(localId).push_back(p);
        allPointsInformation_.at(localId).push_back(W);
    }
    // If EF has not observed point from the current Node, it creates:
    else
    {
        allPlanePoints_.emplace_back(std::deque<Mat31, Eigen::aligned_allocator<Mat31>>());
        allPointsInformation_.emplace_back(std::deque<matData_t>());
        neighbourNodes_.push_back(node);
        node->set_connected_to_EF(true);//This function is required to properly build the L matrix
        nodeIds_.push_back(id);
        uint_t localId = allPlanePoints_.size()-1;
        reverseNodeIds_.emplace(id, localId);
        allPlanePoints_.at(localId).push_back(p);
        allPointsInformation_.at(localId).push_back(W);
        // S and Q are built later, no need to create an element.
    }
    numberPoints_++;

}


void EigenFactorPlaneBase::add_points_array(const MatX &P, std::shared_ptr<Node> &node, mrob::matData_t &W)
{
    assert(P.cols() == 3 && "EigenFactorPlaneBase::add_points_array: Nx3 input is required");
    const auto N = P.rows();
    for (uint_t i = 0; i < N; ++i)
        add_point(P.row(i), node, W);
}

void EigenFactorPlaneBase::add_points_S_matrix(const Mat4 &S, std::shared_ptr<Node> &node, mrob::matData_t &W)
{
    assert(0 && "EigenFactorPlaneBase::add_points_S_matrix: method not implemented");
    // TODO
    std::cout << "EigenFactorPlaneBase::add_points_S_matrix: S =\n" << S << W << std::endl;
    node->print();
}


void EigenFactorPlaneBase::calculate_all_matrices_S()
{
    // processed only once TODO incremetnal additions, if this is ever going to be used?
    if (S_.empty())
    {
        for (auto &vectorPoints: allPlanePoints_)
        {
            Mat4 S = Mat4::Zero();
            for (Mat31 &p : vectorPoints)
            {
                Mat41 pHomog;
                pHomog << p , 1.0;
                S += pHomog * pHomog.transpose();
            }
            S_.emplace_back(S);
        }
        // XXX at this point, we could remove all the points stored, since they will not be used again.
        // Let's keep them for now in case we re-evaluate things or filter, it's just memory.
        allPlanePoints_.clear();
    }
}

void EigenFactorPlaneBase::calculate_all_matrices_Q()
{
    Q_.clear();
    uint_t nodeIdLocal = 0;
    accumulatedQ_ = Mat4::Zero();
    for (auto &S : S_)
    {
        Mat4 T = this->neighbourNodes_[nodeIdLocal]->get_state();
        // Use the corresponding matrix S
        Mat4 Q;
        Q.noalias() =  T * S * T.transpose();
        Q_.push_back(Q);
        accumulatedQ_ += Q;
        nodeIdLocal++;
    }
}

Mat31 EigenFactorPlaneBase::get_mean_point(factor_id_t id)
{
    assert(!S_.empty() && "EigenFactorPlaneBase::get_mean_point: S matrix empty");
    auto localId = reverseNodeIds_.at(id);
    return S_[localId].topRightCorner<3,1>()/S_[localId](3,3);
}

void EigenFactorPlaneBase::print() const
{
    std::cout << "Plane Eigen Factor " <<  this->get_id()
              << " current plane estimated (global coord): " << planeEstimation_.transpose() << std::endl;
    for(auto id : nodeIds_)
        std::cout << "Node ids = "  << id << ", and its reverse in EF = "
                  << reverseNodeIds_.at(id) << std::endl;
    /*for(auto &pc: allPlanePoints_ )
    {
        std::cout << "new pose: \n";
        for (auto &p : pc)
            std::cout << "point = " << p.transpose() <<std::endl;
    }*/
    std::cout << "Plotting S \n";
    for(auto &S: S_)
        std::cout << S << std::endl;
    std::cout << "Plotting Jacobians \n";
    for(auto &J: J_)
        std::cout << J.transpose() << std::endl;
}

MatRefConst EigenFactorPlaneBase::get_jacobian(mrob::factor_id_t id) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlanebase::get_jacobian: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return J_.at(localId);
}


MatRefConst EigenFactorPlaneBase::get_hessian(mrob::factor_id_t id,mrob::factor_id_t /*id_j*/) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlanebase::get_hessian_block: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return H_.at(localId);
}