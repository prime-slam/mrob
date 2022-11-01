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
 * EigenFactorPlaneCenter.hpp
 *
 *  Created on: Oct 7 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

// XXX this duplicate should disappear
#ifndef EIGENFACTORPLANECENTER_HPP_
#define EIGENFACTORPLANECENTER_HPP_


#include "mrob/factor.hpp"
#include "mrob/factors/EigenFactorPlane.hpp"
#include <unordered_map>
#include <deque>
#include <Eigen/StdVector>


namespace mrob{

/**
 * This is a copy of EF Plane for comparison
 * Eigen factor Plane is a vertex that complies with the Fgraph standards
 * and inherits from base EigenFactor.hpp
 *
 * The Plane factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the plane, so the resultant topology
 * is N nodes connecting to the plane factor.
 *
 * NOTE: due to its nature, multiple observation can be added to the same EF,
 * meaning we need to create a constructor PLUS an additional method
 *  - add_point()
 *
 * In order to build the problem we would follow the interface specifications by FGraph
 * but we need extra methods and variables to keep track of the neighbours
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 * XXX should we store all points?
 */
class EigenFactorPlaneCenter: public EigenFactorPlane{
public:
    /**
     * Creates an Eigen Factor plane. The minimum requirements are 1 pose, which is not required
     * at this stage, but will be introduced when we add points/Q matrix.
     */
    EigenFactorPlaneCenter(Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPlaneCenter() override = default;
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
    /**
     * Estimates the plane parameters: v = [n', d]'_{4x1}, where v is unit, (due to the Eigen solution)
     * although for standard plane estimation we could enforce unit on the normal vector n.
     */
    void estimate_plane() override;

    Mat4 accumulatedCenterQ_;//Q matrix of accumulated values for the incremental update of the error.
    Mat4 Tcenter_;

    // subset of pointcloud for the given plane
    //std::unordered_map<factor_id_t, std::vector<Mat31> > allPlanePoints_;

};

}
#endif /* EigenFactorPlaneCenter_HPP_ */
