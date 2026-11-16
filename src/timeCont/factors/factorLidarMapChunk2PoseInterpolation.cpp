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
 * factorLidarMapChunk2PoseInterpolation.cpp
 *
 *  Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Gonzalo Ferrer
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factorLidarMapChunk2PoseInterpolation.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <iostream>
using namespace mrob;

// ---------------------------------------------------------------------------------------------------------------
// PointPairMoments
// ---------------------------------------------------------------------------------------------------------------
void PointPairMoments::clear()
{
    n_ = 0;
    w = 0.0;
    mu.setZero(); q_bar.setZero();
    D.setZero(); C.setZero();
    trE = 0.0;
    p0_.setZero(); q0_.setZero();
    sp_.setZero(); sq_.setZero();
    Spp_.setZero(); Spq_.setZero();
    sqq_ = 0.0;
}

void PointPairMoments::add(const Mat31 &p_imu, const Mat31 &q_world, matData_t weight)
{
    if (n_ == 0)
    {
        p0_ = p_imu;
        q0_ = q_world;
    }
    const Mat31 a = p_imu - p0_;
    const Mat31 b = q_world - q0_;
    w += weight;
    sp_ += weight * a;
    sq_ += weight * b;
    Spp_ += weight * a * a.transpose();
    Spq_ += weight * a * b.transpose();
    sqq_ += weight * b.squaredNorm();
    ++n_;
}

void PointPairMoments::finalize()
{
    if (n_ == 0 || !(w > 0.0))
    {
        w = 0.0;
        return;
    }
    // with a = p - p0, b = q - q0 and the weighted means ma = sp/w, mb = sq/w:  centred moments are
    //   D = Saa - w ma ma^T,  C = Sab - w ma mb^T,  trE = sum w|b|^2 - w |mb|^2
    const Mat31 ma = sp_ / w;
    const Mat31 mb = sq_ / w;
    mu = p0_ + ma;
    q_bar = q0_ + mb;
    D = Spp_ - w * ma * ma.transpose();
    C = Spq_ - w * ma * mb.transpose();
    trE = sqq_ - w * mb.squaredNorm();
}

// ---------------------------------------------------------------------------------------------------------------
// FactorLidarMapChunk2PoseInterpolation
// ---------------------------------------------------------------------------------------------------------------
FactorLidarMapChunk2PoseInterpolation::FactorLidarMapChunk2PoseInterpolation(
        const PointPairMoments &moments,
        matData_t t_obs,
        std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget,
        Factor::robustFactorType robust_type):

        Factor(6,18,robust_type), m_(moments), thau_taw_(0.0)
{
    obs_(0,0) = t_obs;
    r_.setZero();
    W_.setIdentity();
    J_.setZero();
    H6_.setZero();
    g6_.setZero();
    cost_ = 0.0;
    // Order must be preserved
    neighbourNodes_.push_back(nodeOrigin);
    neighbourNodes_.push_back(nodeTarget);
}

void FactorLidarMapChunk2PoseInterpolation::evaluate_residuals()
{
    SE3tc Tx_origin = SE3tc(Mat5(get_neighbour_nodes()->at(0)->get_state()));
    SE3tc Tx_target = SE3tc(Mat5(get_neighbour_nodes()->at(1)->get_state()));
    const matData_t t1 = Tx_origin.time();
    const matData_t t2 = Tx_target.time();
    thau_taw_ = (obs_(0,0) - t1) / (t2 - t1);
    xi_full_target_inv_origin_ = (Tx_origin.inv() * Tx_target).Ln();
    xi_target_inv_origin_ = thau_taw_ * xi_full_target_inv_origin_;
    T_taw_ = Tx_origin * SE3tc(xi_target_inv_origin_);

    // exact cost of the chunk:  sum w|r|^2 = tr D + trE - 2 tr(C R) + w |R mu + p - q_bar|^2
    const Mat3 R = T_taw_.R();
    const Mat31 p = T_taw_.T().block<3,1>(0,3);
    const Mat31 c = R * m_.mu + p - m_.q_bar;
    cost_ = m_.D.trace() + m_.trE - 2.0 * (m_.C * R).trace() + m_.w * c.squaredNorm();
    // the sum is non-negative; round-off may leave a tiny negative value when the chunk fits perfectly
    cost_ = std::max<matData_t>(cost_, 0.0);
}

void FactorLidarMapChunk2PoseInterpolation::evaluate_jacobians()
{
    // Assumes the residuals have been evaluated at the current state.
    const Mat3 R = T_taw_.R();
    const Mat31 p = T_taw_.T().block<3,1>(0,3);
    const Mat31 c = R * m_.mu + p - m_.q_bar;

    // Gauss-Newton normal equations in the tangent (phi, rho) of the interpolated pose, right perturbation.
    const Mat3 CR = m_.C * R;
    Mat31 ax;                                    // axial vector of C R:  (X23-X32, X31-X13, X12-X21)
    ax << CR(1,2) - CR(2,1), CR(2,0) - CR(0,2), CR(0,1) - CR(1,0);
    const Mat31 Rtc = R.transpose() * c;

    H6_.setZero();
    H6_.block<3,3>(0,0) = (m_.D.trace() + m_.w * m_.mu.squaredNorm()) * Mat3::Identity() - m_.D
                          - m_.w * m_.mu * m_.mu.transpose();
    H6_.block<3,3>(0,3) = m_.w * hat3(m_.mu);
    H6_.block<3,3>(3,0) = H6_.block<3,3>(0,3).transpose();
    H6_.block<3,3>(3,3) = m_.w * Mat3::Identity();
    g6_.head<3>() = m_.w * m_.mu.cross(Rtc) - ax;
    g6_.tail<3>() = m_.w * Rtc;

    // Square root of the information:  H = L^T L, pseudo residual r with L^T r = g  (so that J^T J = M^T H M, J^T r = M^T g).
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<matData_t,6,6>> es(H6_);
    const Eigen::Matrix<matData_t,6,1> lam = es.eigenvalues();
    const Eigen::Matrix<matData_t,6,6> V = es.eigenvectors();
    const matData_t tol = 1e-12 * std::max<matData_t>(lam.maxCoeff(), 1e-300);
    Eigen::Matrix<matData_t,6,6> L = Eigen::Matrix<matData_t,6,6>::Zero();
    Eigen::Matrix<matData_t,6,1> r = Eigen::Matrix<matData_t,6,1>::Zero();
    const Eigen::Matrix<matData_t,6,1> Vtg = V.transpose() * g6_;
    for (int k = 0; k < 6; ++k)
    {
        if (lam(k) <= tol) continue;
        const matData_t s = std::sqrt(lam(k));
        L.row(k) = s * V.col(k).transpose();
        r(k) = Vtg(k) / s;
    }
    r_ = r;

    // Interpolated pose -> window poses (same chain rule as FactorLidarMap2PoseInterpolationJacobian):
    //   delta_tau = M1 d1 + M2 d2,   M1 = Ad_{B^-1} - tau Jr(tau xi) Jl^{-1}(xi),   M2 = tau Jr(tau xi) Jr^{-1}(xi)
    const matData_t tau = thau_taw_;
    const SE3tc B_tau = SE3tc(xi_target_inv_origin_);
    const Mat<10,10> Ad_Binv = B_tau.inv().adj();
    const Mat<10,10> Jr_tau  = right_jacobian_tc(xi_target_inv_origin_);
    const Mat<10,10> Jl_inv_xi = inv_left_jacobian_tc(xi_full_target_inv_origin_);
    const Mat<10,10> Jr_inv_xi = inv_right_jacobian_tc(xi_full_target_inv_origin_);
    const Mat<10,10> M1 = Ad_Binv - tau * (Jr_tau * Jl_inv_xi);
    const Mat<10,10> M2 = tau * (Jr_tau * Jr_inv_xi);

    J_.setZero();
    J_.block<6,9>(0,0) = L * M1.topLeftCorner<6,9>();
    J_.block<6,9>(0,9) = L * M2.topLeftCorner<6,9>();
}

void FactorLidarMapChunk2PoseInterpolation::evaluate_chi2()
{
    chi2_ = 0.5 * cost_;
}

void FactorLidarMapChunk2PoseInterpolation::print() const
{
    std::cout << "FactorLidarMapChunk2PoseInterpolation   id = " << id_ << std::endl;
    std::cout << "t: " << obs_(0,0) << "  weight sum: " << m_.w << "  pairs: " << m_.count() << std::endl;
    std::cout << "H6: \n" << H6_ << std::endl;
    std::cout << "g6: " << g6_.transpose() << std::endl;
    std::cout << "chi2_: " << chi2_ << std::endl;
}
