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
 * factorLidarMapChunk2PoseInterpolation.hpp
 *
 *  Author: Ahmed Baza
 *              Ahmed.Baza@skoltech.ru
 *              Gonzalo Ferrer
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTORLIDARMAPCHUNK2POSEINTERPOLATION_HPP_
#define FACTORLIDARMAPCHUNK2POSEINTERPOLATION_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/factor.hpp"

namespace mrob {

/**
 * Sufficient statistics of a set of weighted point pairs (p_i, q_i): p_i is a LiDAR point expressed in the IMU
 * frame, q_i the matched map point in the world frame, w_i > 0 its (scalar) information. All pairs of one chunk
 * are observed at the same time stamp, i.e. under the same interpolated pose T_tau.
 *
 * The statistics are the centred moments
 *      w = sum w_i,   mu = sum w_i p_i / w,   q_bar = sum w_i q_i / w,
 *      D = sum w_i (p_i-mu)(p_i-mu)^T,   C = sum w_i (p_i-mu)(q_i-q_bar)^T,   trE = sum w_i |q_i-q_bar|^2 .
 * They are accumulated in coordinates relative to the first pair (numerical conditioning) and centred in finalize().
 */
class PointPairMoments
{
public:
    PointPairMoments() { clear(); }
    void clear();
    /** Adds one pair. Must be followed by finalize() before the moments are used. */
    void add(const Mat31 &p_imu, const Mat31 &q_world, matData_t weight);
    /** Computes the centred moments from the accumulated sums. */
    void finalize();
    size_t count() const { return n_; }

    matData_t w;     // sum of the weights
    Mat31 mu;        // weighted mean of the points (IMU frame)
    Mat31 q_bar;     // weighted mean of the map points (world frame)
    Mat3 D;          // scatter of the points about mu
    Mat3 C;          // cross-scatter  sum w (p-mu)(q-q_bar)^T
    matData_t trE;   // sum w |q-q_bar|^2

private:
    size_t n_;
    Mat31 p0_, q0_;                 // reference pair (first one added)
    Mat31 sp_, sq_;                 // sums of w a, w b with a = p - p0, b = q - q0
    Mat3 Spp_, Spq_;                // sums of w a a^T, w a b^T
    matData_t sqq_;                 // sum of w |b|^2
};

/**
 * Chunk factor: all LiDAR point-to-point residuals r_i = R_tau p_i + p_tau - q_i of one time stamp tau, as ONE factor.
 *
 * The weighted sum of squares of the chunk depends on the points only through PointPairMoments:
 *      sum w_i |r_i|^2 = tr D + trE - 2 tr(C R) + w |R mu + p - q_bar|^2        (exact)
 * and its Gauss-Newton normal equations (right perturbation of T_tau by (phi, rho)) are
 *      H = [ (tr D + w|mu|^2) I - D - w mu mu^T ,  w [mu]x ;  w [mu]x^T ,  w I ]
 *      g = [ w mu x (R^T c) - ax(C R) ;  w R^T c ],      c = R mu + p - q_bar,
 * identical to the sum of the normal equations of the individual point factors
 * (FactorLidarMap2PoseInterpolationJacobian with information w_i I). H is 6x6 and does not depend on the state.
 *
 * For the factor graph the chunk is presented as a 6-dimensional pseudo-measurement with information I:
 *      J = L [M1 M2]_{(phi,rho)},   r = L^{-T} g,   with L^T L = H (L from the eigen-decomposition of H),
 * so that J^T J = M^T H M and J^T r = M^T g, where M1, M2 are the Jacobians of the interpolated pose with respect to
 * the two window poses (as in FactorLidarMap2PoseInterpolationJacobian). chi2 is the exact cost 1/2 sum w_i |r_i|^2.
 */
class FactorLidarMapChunk2PoseInterpolation : public Factor
{
public:
    FactorLidarMapChunk2PoseInterpolation(const PointPairMoments &moments, matData_t t_obs,
            std::shared_ptr<Node> &nodeOrigin, std::shared_ptr<Node> &nodeTarget,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorLidarMapChunk2PoseInterpolation() = default;

    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;
    void print() const override;
    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J_;}

    /** Gauss-Newton normal equations of the chunk with respect to the interpolated pose, (phi, rho) order. */
    const Mat6 &hessian_pose() const { return H6_; }
    const Mat61 &gradient_pose() const { return g6_; }

protected:
    PointPairMoments m_;
    Mat1 obs_;            // [t] time stamp of the chunk
    Mat61 r_;             // pseudo residual  L^{-T} g
    Mat6 W_;              // identity
    Mat<6,18> J_;         // L [M1 M2]
    Mat6 H6_;
    Mat61 g6_;
    matData_t cost_;      // sum w_i |r_i|^2 (exact)
    SE3tc T_taw_;
    matData_t thau_taw_;
    Mat101 xi_target_inv_origin_;
    Mat101 xi_full_target_inv_origin_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif /* FACTORLIDARMAPCHUNK2POSEINTERPOLATION_HPP_ */
