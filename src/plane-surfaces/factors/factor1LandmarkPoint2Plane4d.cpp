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
 * factor1LandmarkPoint2Plane4d.cpp
 */

#include "mrob/factors/factor1LandmarkPoint2Plane4d.hpp"

#include <iostream>

using namespace mrob;

Factor1LandmarkPoint2Plane4d::Factor1LandmarkPoint2Plane4d(
        std::shared_ptr<Node> &nodeLandmark,
        std::shared_ptr<EigenFactor> &eigenFactorPlane,
        const Mat1 &obsInf,
        Factor::robustFactorType robust_type):
    Factor(1,3, robust_type),
    eigenFactorPlane_(eigenFactorPlane),
    landmark_(Mat31::Zero()),
    plane_(Mat41::Zero()),
    r_(0.0),
    W_(obsInf)
{
    neighbourNodes_.push_back(nodeLandmark);
}

void Factor1LandmarkPoint2Plane4d::evaluate_residuals()
{
    landmark_ = get_neighbour_nodes()->at(0)->get_state();
    eigenFactorPlane_->evaluate_residuals();
    plane_ = eigenFactorPlane_->get_state();

    matData_t normal_norm = plane_.head<3>().norm();
    if (normal_norm > 0.0)
        plane_ /= normal_norm;

    r_ = Mat1(plane_.head<3>().dot(landmark_) + plane_(3));
}

void Factor1LandmarkPoint2Plane4d::evaluate_jacobians()
{
    J_ = plane_.head<3>().transpose();
}

void Factor1LandmarkPoint2Plane4d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1LandmarkPoint2Plane4d::print() const
{
    std::cout << "Printing Factor: " << id_
              << ", landmark = \n" << landmark_
              << "\nplane from EF(" << eigenFactorPlane_->get_id() << ") =\n" << plane_
              << "\n Residuals = \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}
