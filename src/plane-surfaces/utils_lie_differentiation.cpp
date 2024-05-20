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
 * utils_lie_differentiation.cpp
 *
 *  Created on: August 23, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include "mrob/utils_lie_differentiation.hpp"

#include <iostream>

using namespace mrob;


Mat<4,6> mrob::gradient_Q_x_pi(const Mat4 &Q, const Mat41 &pi)
{
    Mat<4,6> jacobian;
    Mat4 dQ;


    // dQ / d xi(0) = [0
    //               -q3
    //                q2
    //                 0]
    dQ.setZero();
    dQ.row(1) << -Q.row(2);
    dQ.row(2) <<  Q.row(1);
    dQ += dQ.transpose().eval();
    jacobian.col(0) = dQ*pi;

    // dQ / d xi(1) = [q3
    //                 0
    //                -q1
    //                 0]
    dQ.setZero();
    dQ.row(0) <<  Q.row(2);
    dQ.row(2) << -Q.row(0);
    dQ += dQ.transpose().eval();
    jacobian.col(1) = dQ*pi;

    // dQ / d xi(2) = [-q2
    //                 q1
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << -Q.row(1);
    dQ.row(1) <<  Q.row(0);
    dQ += dQ.transpose().eval();
    jacobian.col(2) = dQ*pi;

    // dQ / d xi(3) = [q4
    //                 0
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian.col(3) = dQ*pi;

    // dQ / d xi(4) = [0
    //                 q4
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(1) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian.col(4) = dQ*pi;

    // dQ / d xi(5) = [0
    //                 0
    //                 q4
    //                 0]
    dQ.setZero();
    dQ.row(2) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian.col(5) = dQ*pi;


    return jacobian;
}


Mat<3,6> mrob::gradient_Tcenter_Q_x_eta(const Mat4 &T_center, const Mat4 &Q, const Mat31 &eta)
{
    Mat<3,6> jacobian;
    Mat4 dQ;


    // dQ / d xi(0) = [0
    //               -q3
    //                q2
    //                 0]
    dQ.setZero();
    dQ.row(1) << -Q.row(2);
    dQ.row(2) <<  Q.row(1);
    dQ += dQ.transpose().eval();
    //std::cout << "dQ \n" << dQ <<std::endl;
    //std::cout << "dQ left \n" << T_center*dQ <<std::endl;

    dQ = T_center * dQ * T_center.transpose();
    std::cout << "dQ left right \n" << dQ <<std::endl;
    jacobian.col(0) = dQ.topLeftCorner<3,3>()*eta;

    // dQ / d xi(1) = [q3
    //                 0
    //                -q1
    //                 0]
    dQ.setZero();
    dQ.row(0) <<  Q.row(2);
    dQ.row(2) << -Q.row(0);
    dQ += dQ.transpose().eval();
    dQ = T_center * dQ * T_center.transpose();
    jacobian.col(1) = dQ.topLeftCorner<3,3>()*eta;

    // dQ / d xi(2) = [-q2
    //                 q1
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << -Q.row(1);
    dQ.row(1) <<  Q.row(0);
    dQ += dQ.transpose().eval();
    dQ = T_center * dQ * T_center.transpose();
    jacobian.col(2) = dQ.topLeftCorner<3,3>()*eta;

    // dQ / d xi(3) = [q4
    //                 0
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << Q.row(3);
    dQ += dQ.transpose().eval();
    dQ = T_center * dQ * T_center.transpose();
    jacobian.col(3) = dQ.topLeftCorner<3,3>()*eta;

    // dQ / d xi(4) = [0
    //                 q4
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(1) << Q.row(3);
    dQ += dQ.transpose().eval();
    dQ = T_center * dQ * T_center.transpose();
    jacobian.col(4) = dQ.topLeftCorner<3,3>()*eta;

    // dQ / d xi(5) = [0
    //                 0
    //                 q4
    //                 0]
    dQ.setZero();
    dQ.row(2) << Q.row(3);
    dQ += dQ.transpose().eval();
    dQ = T_center * dQ * T_center.transpose();
    jacobian.col(5) = dQ.topLeftCorner<3,3>()*eta;

    return jacobian;
}

Mat6 mrob::pi_t_x_hessian_Q_x_pi(const Mat4 &Q, const Mat41 &pi)
{
    Mat6 hessian = Mat6::Zero();
    Mat41 dQ_x_pi;
    // The Hessian consists of H_Q = 1/2(GiGj + GjGi)Q + 1/2Q(GiGj + GjGi)
    // but when calculating the scalar product pi' * H_Q * pi, due to symmetry
    // only one side of the sum is needed since they are equal.
    // So we will just take one on the summands in H_Q and ignore the 1/2 (equiv x2)
    //
    // It is also necessary to go elemetn by element. For that, we generated a simple
    // code in python doing all cases and reproducing them here.
    //
    // Due to its many zero elemetns and particular structure, the computation of dQ can be
    // simplified into the dot product of some selected rows of Q times pi.
    
    // iter  0 ,  0  =
    // [[ 0.  0.  0.  0.]
    // [ 0. -2.  0.  0.]
    // [ 0.  0. -2.  0.]
    // [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(1) = -2.0*Q.row(1).dot(pi);
    dQ_x_pi(2) = -2.0*Q.row(2).dot(pi);
    hessian(0,0) = pi.dot(dQ_x_pi);

    // iter  0 ,  1  =
    //  [[0. 1. 0. 0.]
    //  [1. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = Q.row(1).dot(pi);
    dQ_x_pi(1) = Q.row(0).dot(pi);
    hessian(0,1) = pi.dot(dQ_x_pi);

    // iter  0 ,  2  =
    //  [[0. 0. 1. 0.]
    //  [0. 0. 0. 0.]
    //  [1. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = Q.row(2).dot(pi);
    dQ_x_pi(2) = Q.row(0).dot(pi);
    hessian(0,2) = pi.dot(dQ_x_pi);

    // iter  0 ,  3  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(0,3) = 0;
    
    // iter  0 ,  4  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 1.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(2) = Q.row(3).dot(pi);
    hessian(0,4) = pi.dot(dQ_x_pi);

    // iter  0 ,  5  =
    //  [[ 0.  0.  0.  0.]
    //  [ 0.  0.  0. -1.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(1) = -Q.row(3).dot(pi);
    hessian(0,5) = pi.dot(dQ_x_pi);

    // iter  1 ,  1  =
    //  [[-2.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0. -2.  0.]
    //  [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = -2.0*Q.row(0).dot(pi);
    dQ_x_pi(2) = -2.0*Q.row(2).dot(pi);
    hessian(1,1) = pi.dot(dQ_x_pi);

    // iter  1 ,  2  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 1. 0.]
    //  [0. 1. 0. 0.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(1) = Q.row(2).dot(pi);
    dQ_x_pi(2) = Q.row(1).dot(pi);
    hessian(1,2) = pi.dot(dQ_x_pi);

    // iter  1 ,  3  =
    //  [[ 0.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0.  0. -1.]
    //  [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(2) = -Q.row(3).dot(pi);
    hessian(1,3) = pi.dot(dQ_x_pi);

    // iter  1 ,  4  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(1,4) = 0.0;

    // iter  1 ,  5  =
    //  [[0. 0. 0. 1.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = Q.row(3).dot(pi);
    hessian(1,5) = pi.dot(dQ_x_pi);

    // iter  2 ,  2  =
    //  [[-2.  0.  0.  0.]
    //  [ 0. -2.  0.  0.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = -2.0*Q.row(0).dot(pi);
    dQ_x_pi(1) = -2.0*Q.row(1).dot(pi);
    hessian(2,2) = pi.dot(dQ_x_pi);

    // iter  2 ,  3  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 1.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(1) = Q.row(3).dot(pi);
    hessian(2,3) = pi.dot(dQ_x_pi);

    // iter  2 ,  4  =
    //  [[ 0.  0.  0. -1.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]
    //  [ 0.  0.  0.  0.]]
    dQ_x_pi.setZero();
    dQ_x_pi(0) = -Q.row(3).dot(pi);
    hessian(2,4) = pi.dot(dQ_x_pi);

    // iter  2 ,  5  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(2,5) = 0.0;

    // iter  3 ,  3  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(3,3) = 0.0;

    // iter  3 ,  4  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(3,4) = 0.0;

    // iter  3 ,  5  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(3,5) = 0.0;

    // iter  4 ,  4  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(4,4) = 0.0;
    
    // iter  4 ,  5  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(4,5) = 0.0;

    // iter  5 ,  5  =
    //  [[0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]
    //  [0. 0. 0. 0.]]
    //hessian(5,5) = 0.0;


    return hessian;
}

Mat<6,4> mrob::pi_t_times_lie_generatives(const Mat41 &pi)
{
    Mat<6,4> gradient = Mat<6,4>::Zero();
    // iter  0  =
    //  [ 0.  3. -2.  0.]
    gradient.row(0) <<     0, pi(2), -pi(1), 0;

    // iter  1  =
    //  [-3.  0.  1.  0.]
    gradient.row(1) << -pi(2),    0, pi(0), 0;

    // iter  2  =
    //  [ 2. -1.  0.  0.]
    gradient.row(2) << pi(1), -pi(0),     0, 0;

    // iter  3  =
    //  [0. 0. 0. 1.]
    gradient.row(3) << 0, 0, 0, pi(0);

    // iter  4  =
    //  [0. 0. 0. 2.]
    gradient.row(4) << 0, 0, 0, pi(1);

    // iter  5  =
    //  [0. 0. 0. 3.]
    gradient.row(5) << 0, 0, 0, pi(2);

    return gradient;

}
