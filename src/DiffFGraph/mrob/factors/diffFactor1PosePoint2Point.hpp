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
 * factor1PosePoint2Point.hpp
 *
 *  Created on: May 28, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef DIFFFACTOR1POSEPOINT2POINT_HPP_
#define DIFFFACTOR1POSEPOINT2POINT_HPP_

#include "mrob/differential_factor.hpp"
#include "mrob/factors/factor1PosePoint2Point.hpp"

namespace mrob{

/**
 * DiffFactor1PoseToPpoint2Point is a vertex in the Fgraph library that models
 * the distribution of observations of a 3D pose. The purpose of this factors is to estimate
 * the relative transformation from two different sets of point clouds, given some planar constraaints.
 *
 * Observations
 *  - Point x
 *  - Point y, from reference y
 *
 * State to estimate is 3D pose y^T_x.
 *
 * The residual, as a convention in the library is:
 *   r = f(x) = (Tx - y) -> 0 if the points are coincident.
 *
 * The Jacobian is then
 *  dr = [-(Tx)^, I]
 *
 */

class DiffFactor1PosePoint2Point : public virtual Factor1PosePoint2Point, public virtual DiffFactor
{
  public:
    DiffFactor1PosePoint2Point(const Mat31 &z_point_x, const Mat31 &z_point_y,  std::shared_ptr<Node> &node,
            const Mat3 &obsInf, Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~DiffFactor1PosePoint2Point() override = default;
    /**
     * All 
     */
    virtual void evaluate_residuals() override;


  protected:
    Mat31 z1_, z2_;
    Mat<3,6> Jxz_;
};

}



#endif /* DIFFFACTOR1POSEPOINT2POINT_HPP_ */
