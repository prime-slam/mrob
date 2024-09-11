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
 * SE3tc.hpp
 *
 *  Created on: September 9, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef SE3TC_HPP_
#define SE3TC_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"


namespace mrob{

/**
 *  \brief Extended Special Euclidean (group) in 3d Time Constrained
 *  Is the group representing rotations, vel and translations:
 *  SE3tc = {T = [R  p  v]  |  R \in SO3 , p \in Re^3, v \in Re^3 , time t \in Re+}
 *                [0 1  0]
 *                [0 t  1]
 *        s.t. dot{p} = v, for any time t.
 *  The Lie Algebra associated to this group is expressed by the coordinates
 *      xi =[omega , acc] \in Re^6, where omega \in Re^3 represents the angular velocity
 *  and acc is the acceleration.
 *  We will preserve this order in this class.
 */

class SE3tc{
    public:
        SE3tc(const Mat5 &T = Mat5::Identity());

        SE3tc(const SE3tc &T);

        SE3tc(const Mat31 &omega, const Mat31 &acc, const matData_t &t);

        SE3tc(const Mat61 &xi, const matData_t &t);

        // this function takes a SE3 lifts to SE3vel with 0 velocity. this is to be used with Ln_position
        SE3tc(const SE3 &T, const matData_t &t);



        Mat31 p() const;
        Mat31 v() const;
        Mat3 R() const;
        Mat4 T_SE3() const;
        Mat5 T() const;
        matData_t t() const;
        Mat<3,5> T_compact() const;


        //Mat6 adj() const;//TODO necesary here?

        void Exp(const Mat61& xi, const matData_t &t);
        Mat61 Ln_position(void) const;
        Mat61 Ln_velocity(void) const;



        void regenerate();
        SE3tc operator*(const mrob::SE3tc& rhs);
        SE3tc& operator=(const SE3tc& rhs);
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

Mat3 integrand_1(const Mat31 &omega, const matData_t &delta_t);
Mat3 integrand_2(const Mat31 &omega, const matData_t &delta_t);

Mat3 inv_integrand_1(const Mat31 &omega, const matData_t &delta_t);
Mat3 inv_integrand_2(const Mat31 &omega, const matData_t &delta_t);

}// end namespace


#endif /* SE3TC_HPP_ */
