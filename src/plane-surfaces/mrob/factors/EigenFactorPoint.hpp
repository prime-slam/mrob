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
 * EigenFactorPoint.hpp
 *
 *  Created on: Oct 24, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef EIGENFACTORPOINT_HPP_
#define EIGENFACTORPOINT_HPP_


#include "mrob/factor.hpp"
#include "mrob/factors/EigenFactorPlaneBase.hpp"
#include <unordered_map>
#include <deque>
#include <Eigen/StdVector>
#include "mrob/SE3.hpp"


namespace mrob{

/**
 * Eigen factor Point is a vertex that complies with the Fgraph standards
 * and inherits from base EigenFactor.hpp and EigenFactorPlane
 *
 * This particualr examples solve the point to point aligments between the centroids of
 * the plane in the current frame vs the overall centroid of the plane:
 *
 * r = W_I*(T0^{-1}*T_i mu_i - mu(Q_0)), where
 *  - i is the current pose,
 *  - mu_i is the centroid of points observed ONLY in the current frame mu_i = col3(T*S_i*T')
 *  - W_i is the weighting (information) which corresponds to the number of points.
 *  - mu(Q_0) is the mean at the first observed PC.
 *
 * This leads to a weighted LSQ
 *
 * The Chi2 is of a different kind than the EFplane, since this is point2point of means
 *
 *
 */
class EigenFactorPoint: public EigenFactorPlaneBase{
public:
    /**
     * Creates an Eigen Factor point.
     */
    EigenFactorPoint(Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPoint() = default;
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
     * Chi2 is a scaling of the plane error
     */
    void evaluate_chi2() override;


protected:
    void estimate_plane() override {};
    //vector of residuals, and T*mu_i follows the same indexing than Jacobian and Hessian
    std::deque<Mat31, Eigen::aligned_allocator<Mat31>> r_,transformed_mu_;
    SE3 T_ini_inv_;

};

}
#endif /* EigenFactorPlane_HPP_ */
