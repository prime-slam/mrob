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
 * PiFactorPlane.cpp
 *
 *  Created on: Oct 26, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/PiFactorPlane.hpp"

#include <iostream>
#include <Eigen/Cholesky>
#include "mrob/SE3.hpp"

using namespace mrob;

PiFactorPlane::PiFactorPlane(std::shared_ptr<Node> &nodePlane,Factor::robustFactorType robust_type):
        EigenFactorPlane(robust_type)
{
    this->set_dim_obs(4);// This is as we observe N homogenous points, but contracted to a vector
    // add plane
    neighbourNodes_.push_back(nodePlane);// This interferes with the poses later on the ordering?
}

void PiFactorPlane::evaluate_residuals()
{
    // First calculates the matrices
    calculate_all_matrices_S();
    calculate_squared_all_matrices_S();

    // the residual is a vector of dimension 4xN(number of poses)
    uint_t N = get_all_nodes_dim() - 4;//remove node plane -> dim =4
    residual_.clear();

    // Iterate over poses, strict order appearing in here
}

void PiFactorPlane::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    Jacobian_.clear();

    // dr/dxi = sqrt(S)'* T' * G' * pi = sqrt(S)'* T' * [normal^ | 0 0 0  ]
    //                                                   0 0 0   | normal']

    //dr/dpi = sqrt(S)'* T'
}

void PiFactorPlane::evaluate_chi2()
{
    chi2_ = 0.5 * residual_.dot(residual_);// wieghting already included in the residual by sqrt(S)'
}

void PiFactorPlane::calculate_squared_all_matrices_S()
{
    if (sqrtTransposeS_.empty())
    {
        Mat4 sqrtS = Mat4::Zero();
        for (auto &S : S_)
        {
            sqrtS = S.llt().matrixL().transpose();
            sqrtTransposeS_.emplace_back();
        }

    }
}



