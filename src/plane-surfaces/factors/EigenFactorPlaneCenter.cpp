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
 * plane_factor.cpp
 *
 *  Created on: Aug 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlaneCenter.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"

using namespace mrob;

EigenFactorPlaneCenter::EigenFactorPlaneCenter(Factor::robustFactorType robust_type):
        EigenFactor(robust_type),
        Tcenter_(Mat4::Identity()),
        planeEstimation_{Mat41::Zero()},
        planeEstimationUnit_{Mat41::Zero()},
        planeError_{0.0},
        numberPoints_{0}
{
}

void EigenFactorPlaneCenter::evaluate_residuals()
{
    this->estimate_plane();
}

void EigenFactorPlaneCenter::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    for (auto &Qt: Q_)
    {
        Mat61 jacobian = Mat61::Zero();
        Mat6 hessian = Mat6::Zero();
        Mat4 dQ = Mat4::Zero();
        for (uint_t i = 0 ; i < 6; i++)
        {
            dQ = SE3GenerativeMatrix(i)*Qt + Qt*SE3GenerativeMatrix(i).transpose();
            Mat4 dQcenter = Tcenter_ * dQ * Tcenter_.transpose();
            jacobian(i) = planeEstimationUnit_.dot(dQcenter*planeEstimationUnit_);

            //now calculate Hessian here. Upper triangular view
            Mat4 ddQ; // second derivative of the Q matrix
            for (uint_t j = i ; j< 6 ; ++j)
            {
                ddQ.setZero();
                ddQ = SE3GenerativeMatrix(i)*SE3GenerativeMatrix(j) + SE3GenerativeMatrix(j)*SE3GenerativeMatrix(i);
                //compound operator *= as in a*=b (this multiplies on the right: a*=b is equivalent to a = a*b)
                ddQ *= 0.5 * Qt;
                ddQ += SE3GenerativeMatrix(j) * dQ;//here indices should be different, later Hessian is symmetric.
                ddQ += ddQ.transpose().eval();
                // Transformation, translation for center the matrix derivative is applied here
                ddQ = Tcenter_ * ddQ * Tcenter_.transpose();
                hessian(i,j) = planeEstimationUnit_.dot(ddQ*planeEstimationUnit_);
            }
        }
        J_.push_back(jacobian);
        H_.push_back(hessian);

    }
}

void EigenFactorPlaneCenter::evaluate_chi2()
{
    chi2_ = planeError_;//XXX this is not the exact plane error, but requires chi2 = 1/2 pi' Q pi
}

void EigenFactorPlaneCenter::add_point(const Mat31& p, std::shared_ptr<Node> &node, matData_t &W)
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


double EigenFactorPlaneCenter::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (auto &Qt: Q_)
    {
        accumulatedQ_ += Qt;
    }

    // Center the plane requires a transformation (translation) such that
    // pi_centered = [n, 0] so, T * pi, where T = [I, -n d].
    // and n d = - sum{p} / N = -E{x}    from the centered calculation of a plane
    Tcenter_.topRightCorner<3,1>() =  -accumulatedQ_.topRightCorner<3,1>()/accumulatedQ_(3,3);
    //std::cout << "T center = " << Tcenter_ <<  std::endl;

    //std::cout << "Q= \n" << accumulatedQ_ <<  std::endl;

    accumulatedCenterQ_ = Tcenter_ * accumulatedQ_ * Tcenter_.transpose();

    //std::cout << "new function Q= \n" << accumulatedCenterQ_ <<  std::endl;


    // Only needs Lower View from Q (https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html)
    Eigen::SelfAdjointEigenSolver<Mat3> es;
    es.computeDirect(accumulatedCenterQ_.topLeftCorner<3,3>());
    planeEstimationUnit_.head<3>() = es.eigenvectors().col(0);
    planeEstimationUnit_(3) = 0.0;

    //std::cout << "\n and solution plane = \n" << planeEstimationUnit_ <<  std::endl;
    //std::cout << "plane estimation error (0): " << es.eigenvalues() <<  std::endl;

    //planeError_ = planeEstimation_.dot(accumulatedQ_*planeEstimation_);
    planeError_ = es.eigenvalues()(0);
    //std::cout << "plane estimation error method 3 = " << planeError_ << ", plane = " << planeEstimation_ << std::endl;

    return planeError_;
}

void EigenFactorPlaneCenter::calculate_all_matrices_S(bool reset)
{
    if (reset)
        S_.clear();
    if (S_.empty())
    {
        for (auto &vectorPoints: allPlanePoints_)
        {
            Mat4 S = Mat4::Zero();
            for (Mat31 &p : vectorPoints)
            {
                Mat41 pHomog;
                pHomog << p , 1.0;
                S += pHomog * pHomog.transpose();//TODO robust: add coeficient W here...
            }
            S_.push_back(S);
        }
    }
}

void EigenFactorPlaneCenter::calculate_all_matrices_Q()
{
    Q_.clear();
    uint_t nodeIdLocal = 0;
    for (auto &S : S_)
    {
        Mat4 T = this->neighbourNodes_[nodeIdLocal]->get_state();
        // Use the corresponding matrix S
        Mat4 Q;
        Q.noalias() =  T * S * T.transpose();
        Q_.push_back(Q);
        nodeIdLocal++;
    }
}

Mat31 EigenFactorPlaneCenter::get_mean_point(factor_id_t id)
{
    assert(!S_.empty() && "EigenFactorPlaneCenter::get_mean_point: S matrix empty");
    auto localId = reverseNodeIds_.at(id);
    return S_[localId].topRightCorner<3,1>()/S_[localId](3,3);
}

void EigenFactorPlaneCenter::print() const
{
    std::cout << "Plane Eigen Factor " <<  this->get_id()
              << " current plane estimated: " << planeEstimationUnit_.transpose() << std::endl;
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


MatRefConst EigenFactorPlaneCenter::get_jacobian(mrob::factor_id_t id) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlaneCenter::get_jacobian: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return J_.at(localId);
}

MatRefConst EigenFactorPlaneCenter::get_hessian(mrob::factor_id_t id) const
{
    assert(reverseNodeIds_.count(id)   && "EigenFactorPlaneCenter::get_hessian: element not found");
    uint_t localId = reverseNodeIds_.at(id);
    return H_.at(localId);
}

