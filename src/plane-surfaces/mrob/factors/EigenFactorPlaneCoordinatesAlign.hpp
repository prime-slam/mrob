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
 * EigenFactorPlaneCoordinatesAlign.hpp
 *
 *  Created on: Oct 31 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

// XXX this duplicate should disappear
#ifndef EIGENFACTORPLANECOORDINATEALIGN_HPP_
#define EIGENFACTORPLANECOORDINATEALIGN_HPP_


#include "mrob/factor.hpp"
#include "mrob/factors/EigenFactorPlaneCenter.hpp"
#include <unordered_map>
#include <deque>
#include <Eigen/StdVector>


namespace mrob{

/**
 * This is an implementation of the Plane Coordinates Align (BA,multiPC) from Huang RAL2021
 *
 * It requires the estimation of planes from one side and then the matching of these parameters with
 * the accumulated matrix S. The EF strucutre is very convenient to use here
 */
class EigenFactorPlaneCoordinatesAlign: public EigenFactorPlaneCenter{
public:
    /**
     * Creates an Eigen Factor plane. The minimum requirements are 1 pose, which is not required
     * at this stage, but will be introduced when we add points/Q matrix.
     */
    EigenFactorPlaneCoordinatesAlign(Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPlaneCoordinatesAlign() = default;
    /**
     * Jacobians are not evaluated, just the residuals.
     * This function is calculating the current plane estimation
     */
    void evaluate_residuals() override;
    /**
     * Evaluates Jacobians, given the residual evaluated
     * It also evaluated the Hessians
     */
    void evaluate_jacobians() override;
    /**
     * Chi2 is a scaling of the plane error from lambda_min
     */
    void evaluate_chi2() override;


protected:
    Mat31 get_estimate_normal() const;

    Mat31 get_estimate_mean() const;

    void estimate_planes_at_poses();

    // These are the vectors of the eigenV for each pose of the plane, to be later used
    std::deque<matData_t> lambda_1_,lambda_2_;//eigen values where l1 > l2> l3, to follow their notation
    std::deque<matData_t> r1_, r2_, r3_, n_points_;//residual vectors plus point/plane at k pose
    std::deque<Mat31, Eigen::aligned_allocator<Mat31>> v1_, v2_;// eigen vectors, same ordering


};

}
#endif /* EigenFactorPlaneCenter_HPP_ */
