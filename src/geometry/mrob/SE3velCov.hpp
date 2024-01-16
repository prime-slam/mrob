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
 * SE3velCov.hpp
 *
 *  Created on: June 12, 2021
 *      Author: Aleksei Panchenko
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SE3VELCOV_HPP_
#define SE3VELCOV_HPP_

#include "mrob/SE3vel.hpp"
#include "mrob/SE3cov.hpp"

namespace mrob
{
    class SE3velCov : public mrob::SE3vel
    {
    // TODO: this class needs deep review, since the parent class has been modified, on reversing the order of the 
    // Lie coodinates to be [theta,t,v]. This affected:
    //  - Contructor
    //  - Ln, Exp
    //  - Adjoint
    public:
        SE3velCov(void);
        SE3velCov(const SE3vel &pose, const Mat9 &covariance);
        Mat9 covariance;
        void compound_2nd_order(const SE3vel &pose_increment, const Mat9 &increment_covariance, const double dt);
        void compound_4th_order(const SE3vel &pose_increment, const Mat9 &increment_covariance, const double dt);

        Mat9 getQ(const Mat3& cov_a, const Mat3& cov_w, const double dt) const;

        Mat9 get_S_4th(const Mat9 &Q) const;

        void print();

        // transforms covariance matrix to notation from Barfoot's papers
        // transform(transform(cov)) = cov - self-inverse
        Mat9 transform(const Mat9 &covariance) const;
    };

} // end namespace

#endif //SE3VELCOV_HPP_
