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
 * SE3tc.cpp
 *
 *  Created on: September 9, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/SE3tc.hpp"
#include <Eigen/LU>

#include <iostream>

using namespace mrob;

SE3tc::SE3tc(const Mat5 &T) : T_(T) {}

SE3tc::SE3tc(const Mat5 &T, const matData_t &t) : T_(T)
{
    this->set_time(t);
}

SE3tc::SE3tc(const SE3tc &T) : T_(T.T()){}

SE3tc::SE3tc(const Mat31 &omega, const Mat31 &acc, const matData_t &t) : T_(Mat5::Identity())
{
    Mat101 xi = Mat101::Zero();
    xi(9) = t;
    xi.head(3) = omega * t;
    xi.segment<3>(6) = acc * t;
    // std::cout << "xi = \n" << xi << std::endl;
    this->Exp(xi);
}



SE3tc::SE3tc(const Mat91 &xi, const matData_t &t): T_(Mat5::Identity())
{
    // xi- = [omega*t, vel*t, acc*t] and t
    Mat101 xi_plus;
    xi_plus << xi, t;
    this->Exp(xi_plus);
}

SE3tc::SE3tc(const Mat101 &xi): T_(Mat5::Identity())
{
    // xi = [omega*t, vel*t, acc*t, t]
    this->Exp(xi);
}


SE3tc::SE3tc(const SE3 &T, const matData_t &t): T_(Mat5::Identity())
{
    T_.topLeftCorner<3,3>() << T.R();

    Mat31 translation = T.p();
    T_.block<3,1>(0,3) = translation;
    this->set_time(t);
    return;
}

SE3tc::SE3tc(const SE3vel &T, const matData_t &t): T_(T.T())
{
    this->set_time(t);
}

Mat31 SE3tc::p() const
{
    return T_.block<3,1>(0,3);
}

Mat31 SE3tc::v() const
{
    return T_.topRightCorner<3,1>();
}

Mat3 SE3tc::R() const
{
    return T_.topLeftCorner<3,3>();
}

const Eigen::Ref<const Mat5> SE3tc::T(void) const
{
    return T_;
}

Mat4 SE3tc::T_SE3(void) const
{
    return T_.topLeftCorner<4,4>();
}

matData_t SE3tc::time(void) const
{
    return this->T_(4,3);//time
}

void SE3tc::set_time(const matData_t &time_stamp)
{
    T_(4,3) = time_stamp;
}

Mat<3,5> SE3tc::T_compact() const
{
    return T_.topLeftCorner<3,5>();
}

SE3vel SE3tc::vel() const
{
    return SE3vel(*this);
}

SE3tc SE3tc::operator*(const SE3tc& rhs)
{
    return SE3tc(Mat5(T_*rhs.T()));
}

SE3tc& SE3tc::operator=(const SE3tc& rhs)
{
    // check for self assignment TODO
    if (this == &rhs)
        return *this;
    T_ = rhs.T();
    return *this;
}

void SE3tc::Exp(const Mat101& xi)  
{
    Mat5 result(Mat5::Identity());

    Mat31 phi = xi.head(3);
    Mat31 pos = xi.segment<3>(3);
    Mat31 vel = xi.segment<3>(6);
    matData_t t = xi(9);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();

    Mat3 integ1;
    integ1 << left_jacobian(phi);
    result.block<3,1>(0,3) << integ1*pos + left_jacobian_2_so3(phi)*vel*t;
    result.block<3,1>(0,4) << integ1*vel;
    
    result(4,3) = t;

    this->T_ = result;
}

Mat101 SE3tc::Ln() const
{
    Mat101 result(Mat101::Zero());

    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 p = this->p();
    matData_t delta_t = this->time();
    
    SO3 tmp(R);
    Mat31 phi = tmp.ln_vee();
    Mat3 inv_int_1 = inv_left_jacobian(phi);
    Mat3 int_2 = left_jacobian_2_so3(phi);

    result.head(3) << phi;
    result.segment<3>(3) << inv_int_1*p - inv_int_1 * int_2 * inv_int_1 * (delta_t * v);
    result.segment<3>(6) << inv_int_1*v;
    result(9) = delta_t;

    return result;
}


Mat61 SE3tc::Ln_from_position_to_omega_acc() const
{
    Mat61 result(Mat61::Zero());

    Mat3 R = this->R();
    Mat31 p = this->p();
    matData_t delta_t = this->time();

    if (std::fabs(delta_t) < 1e-8)
    {
        return result;
    }

    SO3 tmp(R);
    Mat31 phi = tmp.ln_vee();
    Mat3 jac = inv_left_jacobian_2_so3(phi);
    matData_t delta_t2 = delta_t*delta_t;

    result.head(3) << phi/delta_t;
    result.tail(3) << jac*p/delta_t2;

    return result;
}


SE3tc SE3tc::inv(void) const
{
    Mat5 inv;
    Mat3 R = this->R();
    R.transposeInPlace();
    inv << R, R * ( -this->p() + this->time()*this->v()), -R*this->v(),
           0,0,0,1,0, 
           0,0,0,-this->time(),1;
    return SE3tc(inv);

}


void SE3tc::regenerate()
{
    Mat101 xi = this->Ln();
    this->Exp(xi);
}


Mat<10,10> SE3tc::adj() const
{
    Mat<10,10> res(Mat<10,10>::Zero());
    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 p = this->p();
    matData_t t = this->time();


    res.block<3,3>(0,0) = R;
    res.block<3,3>(3,3) = R;
    res.block<3,3>(6,6) = R;

    res.block<3,3>(3,0) = hat3(p-t*v)*R;
    res.block<3,3>(6,0) = hat3(v)*R;

    res.block<3,3>(3,6) = -t*R;
    res.block<3,1>(3,9) = v;
    res(9,9) = 1.0;
    return res;
}

Mat<9,9> SE3tc::adj_vel() const
{
    Mat<9,9> res;
    Mat<10,10> adjoint;
    adjoint = this->adj();
    res = adjoint.topLeftCorner<9,9>();
    return res;
}

void SE3tc::print() const
{
    std::cout << T_ << std::endl;
}


std::ostream& SE3tc::operator<<(std::ostream &os)
{
    os << T_;
    return os;
}

std::string SE3tc::toString() const
{
    std::stringstream ss;
    ss << this->T_;
    return ss.str();
}



Mat3 mrob::left_jacobian_2_so3(const Mat31 &phi)
{
    Mat3 result = Mat3::Identity()*0.5;
    Mat3 phi_hat = hat3(phi);
    double o = phi.norm();
    double o2 = phi.squaredNorm();
    // If rotation is not zero
    matData_t c3, c4;
    if ( o > 1e-3){
        c3 = (o - std::sin(o))/o2/o;
        c4 = (std::cos(o) - 1.0 + 0.5*o2)/o2/o2;
    }
    else
    {
        // second order Taylor (first order is zero since this is an even function)
        c3 = 1.0/6.0 - o2/120;
        // Second order Taylor
        c4 = 1.0/24.0 - o2/720;
    }
    result += c3*phi_hat + c4*phi_hat*phi_hat;


    return result;
}


Mat3 mrob::inv_left_jacobian_2_so3(const Mat31 &phi)
{
    Mat3 result;
    Mat3 left_jacobian = mrob::left_jacobian_2_so3(phi);
    result = left_jacobian.inverse();// There is a close-form

    return result;
}

Mat<10,10> mrob::inv_left_jacobian_tc(const Mat101 &xi)
{
    Mat<10,10> result(Mat<10,10>::Zero());
    result(9,9) = 1.0;
    Mat31 theta = xi.head(3);
    Mat31 rho = xi.segment<3>(3);
    Mat31 nu = xi.segment<3>(6);
    matData_t t = xi(9);


    // Diagonal block, corresponding to the inverse left Jacobian of the rotation
    Mat3 inverse_J_R;
    inverse_J_R = mrob::inv_left_jacobian(theta);
    result.block<3,3>(0,0) = inverse_J_R;
    result.block<3,3>(3,3) = inverse_J_R;
    result.block<3,3>(6,6) = inverse_J_R;

    Mat3 Qinv;
    Qinv = mrob::Q_in_SE3invJacobian(theta,nu);
    result.block<3,3>(6,0) = Qinv;

    // M (theta nu) block matrix
    Mat3 M;
    matData_t theta_norm = theta.norm();
    matData_t theta_norm_2 = theta.dot(theta);
    M  = -0.5*Mat3::Identity();
    Mat3 theta_hat;
    theta_hat = hat3(theta);
    matData_t alpha_M;
    matData_t tan_norm = std::tan(theta_norm*0.5);
    matData_t inv_theta_norm = 1/theta_norm;
    matData_t inv_theta_norm_2 = 1/theta_norm_2;
    if (theta_norm > 1e-4)
    {
        alpha_M = inv_theta_norm_2 - 0.5 * inv_theta_norm /tan_norm;
    }
    else
    {
        alpha_M = 0.0833333 + 0.00138889 * theta_norm_2;
    }
    M  += alpha_M  * theta_hat;
    result.block<3,1>(3,9) = M * nu;

    // L (theta, t) block matrix
    Mat3 L;
    L = 0.5*t*Mat3::Identity();
    matData_t alpha_L;
    matData_t sin2_norm = std::sin(theta_norm*0.5);
    sin2_norm *= sin2_norm;
    if (theta_norm > 1e-4)
    {
        alpha_L =  t * inv_theta_norm * (0.5/tan_norm  - theta_norm * 0.25 / sin2_norm );
    }
    else
    {
        alpha_L = t*(-0.166667 - 0.00555556 * theta_norm_2);
    }
    L += alpha_L * theta_hat;
    result.block<3,3>(3,6) = L;

    // P (theta, rho, nu, t) block matrix
    Mat3 P;
    P = mrob::Q_in_SE3invJacobian(theta,rho);
    Mat3 nu_hat;
    nu_hat = hat3(nu);
    P -= t /12 * nu_hat;
    
    Mat3 theta_hat_2;
    theta_hat_2 = theta_hat * theta_hat;
    Mat3 nu_theta2, theta2_nu;
    nu_theta2 = nu_hat * theta_hat_2;
    theta2_nu = theta_hat_2 * nu_hat;
    matData_t C_1,C_2,C_3,C_4;
    if (theta_norm > 1e-4)
    {
        C_1 = -t * inv_theta_norm_2 * ( 0.5*inv_theta_norm / tan_norm  - inv_theta_norm_2 + 1.0/12.0);
        C_2 = -alpha_L*inv_theta_norm_2;
        C_2 += 0.5*t * inv_theta_norm_2 * inv_theta_norm / tan_norm  - (1.0/12.0)*t*inv_theta_norm_2 - t * inv_theta_norm_2 * inv_theta_norm_2;
    }
    else
    {
        C_1 = t * (0.00138889 + 0.0000330688 * theta_norm_2);
        C_2 = 0.0;//set to zero
    }
    P += C_1*nu_theta2 + C_2 * theta2_nu;
    
    //C_3 = -2*t*BN[4]/np.math.factorial(4) + 6*t*BN[6]/np.math.factorial(6)*theta_norm_2
    C_3 = t*0.0027777777777729926 + t*0.00019841269841268632*theta_norm_2;
    //C_4 = -3*t*BN[6]/np.math.factorial(6) + 8*t*BN[8]/np.math.factorial(8)*theta_norm_2
    C_4 = -t*9.920634920634316e-05 - t*6.613756613756549e-06*theta_norm_2;
    P +=  theta_hat * nu_hat * theta_hat * C_3;
    P += theta_hat_2 * nu_hat * theta_hat_2 * C_4;
    result.block<3,3>(3,0) = P;



    return result;
}


Mat<10,10> mrob::inv_right_jacobian_tc(const Mat101 &xi)
{
    Mat101 xi_negative = -xi;
    xi_negative(9) = xi(9); // time is not negative, this is due to the constrain on the coordinates
    return inv_left_jacobian_tc(xi_negative);
}

Mat<10,10> mrob::left_jacobian_tc(const Mat101 &xi)
{
    Mat<10,10> result(Mat<10,10>::Zero());
    
    return result;
}

Mat<10,10> mrob::right_jacobian_tc(const Mat101 &xi)
{
    Mat101 xi_negative = -xi;
    xi_negative(9) = xi(9); // time is not negative, this is due to the constrain on the coordinates
    return left_jacobian_tc(xi_negative);
}
