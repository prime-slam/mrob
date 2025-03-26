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
#include "mrob/SE3vel.hpp"


namespace mrob{



/**
 *  \brief Extended Special Euclidean (group) in 3d Time Constrained
 *  Is the group representing rotations, vel and translations:
 *  SE3tc = {T = [R  p  v]  |  R \in SO3 , p \in Re^3, v \in Re^3 , time t \in Re+}
 *                [0 1  0]
 *                [0 t  1]
 *        s.t. dot{p} = v, for any time t when conditions are met
 *  The Lie Algebra associated to this group is expressed by the coordinates
 *      xi =[omega*t , vel*t, acc*t, t] \in Re^10, where 
 *          - t is the time interval for integration
 *          - omega \in Re^3 represents the angular velocity
 *          - vel is velocity and acc is the acceleration.
 *  We will preserve this order in this class.
 * 
 * NOTE:
 * When vel = 0, the the constraint dot{p} = v is fullfilled
 * When acc = 0, this equals the SE(3)
 */

class SE3vel;

class SE3tc{
    public:
        SE3tc(const Mat5 &T = Mat5::Identity());

        SE3tc(const Mat5 &T, const matData_t &t);

        SE3tc(const SE3tc &T);

        SE3tc(const Mat31 &omega, const Mat31 &acc, const matData_t &t);

        SE3tc(const Mat91 &xi, const matData_t &t);

        SE3tc(const Mat101 &xi);

        // this function takes a SE3 lifts to SE3vel with 0 velocity
        SE3tc(const SE3 &T, const matData_t &t);

        // This constructor simply takes a SE3vel and ads the time, converting it into a tc object
        SE3tc(const SE3vel &T, const matData_t &t);



        Mat31 p() const;
        Mat31 v() const;
        Mat3 R() const;
        Mat4 T_SE3() const;
        Mat5 T() const;
        matData_t t() const;
        Mat<3,5> T_compact() const;

        // The adjoint coordinates plus the contraint -> dim 10.
        Mat<10,10> adj() const;

        // The adjoint when transforming the action T_SE3tc * Exp_SE3vel
        //XXX deprecated? who uses this
        Mat<9,9> adj_vel() const;

        SE3tc inv(void) const;

        void Exp(const Mat101& xi);
        Mat101 Ln(void) const;
        Mat61 Ln_position(void) const;

        SE3vel vel() const;

        
        
        
        void set_time(const matData_t &time_stamp =0.0);// this method is used for updates 9such as optimization) where the update is an instantaneous correction of the pose.
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

    // Order 2 (double integration) of the Jacobian
    Mat3 left_jacobian_2(const Mat31 &phi);
    
    // Inverse Left Jacobian of the SE3tc
    Mat<10,10> inv_left_jacobian_tc(const Mat101 &xi);


}// end namespace


#endif /* SE3TC_HPP_ */
