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
 * factorCameraProj3dPoint.hpp
 *
 *  Created on: March 13, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTORCAMERAPROJ3DPOINT_HPP_
#define FACTORCAMERAPROJ3DPOINT_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The FactorCameraProj3dPoint is a vertex representing the distribution between
 * a Rigid Body Transformation encoding a 3D pose and a Landmark, a 3D point
 * projected to the image plane in 2D.
 *
 * The observation is a 2D point, in the local frame of the current 3D pose,
 * the sensor reference frame
 * The two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's, are:
 *  - 1 Pose in 3d (Camera)
 *  - 1 Landmark3d (Point)
 *  - (TODO) Camera intrinsics K
 * We provide the node's Id to get the correspondent Jacobian
 *
 *
 * In particular, the relation between the transformation of poses is:
 *   z = proj_K(T^{-1}*l),   where proj_K(p) = K * [x/z, y/z, 1]'
 *
 * z is a 2d pixel coordinates in the camera plane
 * T is the transformation encoded by the 3D pose, the local frame (sensor)
 * l is a 3d point encoding the landmark position in the map coordinate frame
 *
 * and the residual is thus:
 *   r = proj_K(T^{-1}l) - z
 *
 *
 */

class FactorCameraProj3dPoint : public Factor
{
  public:
    FactorCameraProj3dPoint(const Mat21 &observation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodeLandmark,
            const Mat41 &camera_k,
            const Mat2 &obsInf = Mat2::Identity(),
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorCameraProj3dPoint() = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const override;

    MatRefConst get_obs() const override {return obs_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian(mrob::factor_id_t /*id = 0*/) const override {return J_;};

  protected:
    Mat21 obs_, r_; // Assumes this observations has been undistorted
    Mat31 landmark_, local_point_;
    Mat41 camera_k_; // This encodes [fx, fy, cx, cy]
    SE3 Tinv_;
    Mat2 W_;//inverse of observation covariance (information matrix)
    Mat<2,9> J_;//Joint Jacobian: 6 (pose) + 3(pint) || + 4 (K)
    bool reversedNodeOrder_;//flag to keep order when building the adjacency matrix. This should be transparent for the user

    //project_point in 3D to 2D by the camera parameters in this class
    //TODO, maybe move projective methods all together? this can be used in many other places in visual
    Mat21 project_point(const Mat31 point);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTORCAMERAPROJ3DPOINT_HPP_ */