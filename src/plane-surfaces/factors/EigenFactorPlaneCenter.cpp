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
        EigenFactorPlane(robust_type),
        Tcenter_(Mat4::Identity()),
        planeEstimationUnit_{Mat41::Zero()}
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




