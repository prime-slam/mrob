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
 * EigenFactorPlane.hpp
 *
 *  Created on: Oct 26 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef PIFACTORPLANE_HPP_
#define PIFACTORPLABE_HPP_


#include "mrob/factor.hpp"
#include "mrob/factors/EigenFactorPlane.hpp"
#include <unordered_map>
#include <deque>
#include <Eigen/StdVector>


namespace mrob{

/**
 * Reimplementation of pi-factor from Zhou et al ICRA 2021
 * This factor is mostly based on the EF, using the squared root form of the S matrix
 * and connects two nodes, the pose and the plane landmark
 *
 * sum ||pi' T p_i||^2 = || \sqrt(S)' T' pi||^2
 *
 * the two nodes connected at T (pose) and pi (plane 4d)
 * The observation is the sqrt(S), which in clude all points observed
 */
class PiFactorPlane: public EigenFactorPlane{
public:
    /**
     * Creates the Pi factor.
     *
     * As in EF, observations are points, included later, but it requires a node plane 4d to be added
     */
    PiFactorPlane(std::shared_ptr<Node> &nodePlane,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~PiFactorPlane() override = default;
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
    void calculate_squared_all_matrices_S();
    // Since we dont know a priori the dimensions, we grow as more nodes are added
    MatX Jacobian_;
    MatX1 residual_;

    std::deque<Mat4, Eigen::aligned_allocator<Mat4>> sqrtTransposeS_;

};

}
#endif /* PiFactorPlane_HPP_ */
