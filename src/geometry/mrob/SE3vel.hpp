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
 * SE3vel.hpp
 *
 *  Created on: June 12, 2021
 *      Author: Aleksei Panchenko
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SE3VEL_HPP_
#define SE3VEL_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"

namespace mrob{

/**
 *  \brief Extended Special Euclidean (group) in 3d
 *  Is the group representing rotations, vel and translations:
 *  SE3vel = {T = [R  t  v]  |  R \in SO3 , t \in Re^3, v \in Re^3 }
 *                [0   I  ]
 *  The Lie Algebra associated to this group is expressed by the coordinates
 *      xi =[theta , pho, vel] \in Re^9, where theta \in Re^3 represents the rotation
 *  and pho the translation and vel the velocity.
 *  We will preserve this order in this class.
 */

class SE3vel{
    public:
        SE3vel(const Mat5 &T = Mat5::Identity());

        SE3vel(const SE3vel &T);

        SE3vel(const SO3 &R, const Mat31 &t, const Mat31 &v);

        SE3vel(const Mat91 &xi);

        SE3vel inv(void) const;

        Mat31 t() const;
        Mat31 v() const;
        Mat3 R() const;
        Mat5 T() const;
        Mat<3,5> T_compact() const;


        Mat9 adj() const;

        void Exp(const Mat91 &xi);
        Mat91 Ln(void) const;

        void regenerate();
        SE3vel operator*(const mrob::SE3vel& rhs);
        SE3vel& operator=(const SE3vel& rhs);
        std::ostream& operator<<(std::ostream &os);
        /**
         * @brief Generates string representation of the object
         *
         * @return std::string object to print
         */
        std::string toString() const;

        void print() const;
     
    protected:
        Mat5 T_;
};

Mat5 hat9(const Mat91 &xi);

Mat91 vee9(const Mat5 &xi_hat);

}// end namespace


#endif /* SE3VEL_HPP_ */
