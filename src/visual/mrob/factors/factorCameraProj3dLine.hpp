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
 * factorCameraProj3dLine.hpp
 *
 *  Created on: March 13, 2023
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTORCAMERAPROJ3DLINE_HPP_
#define FACTORCAMERAPROJ3DLINE_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The FactorCameraProj3dLine is a vertex representing the distribution between
 * a Rigid Body Transformation encoding a 3D pose and a 3D line expressed as 2 3D points,
 * projected to the image plane in 2D (Vakhitov, ECCV 2016)
 *
 * The observation is a 2D line, in the local frame of the current 3D pose,
 * the sensor reference frame
 * The two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's, are:
 *  - 2 observation (1,2) pixel coordinates in the image with start-end segments of the line
 *  - 1 Pose in 3d (Camera)
 *  - 2 Points in 3D (map coordinates) expressing the line end-start position
 *  - (TODO) Camera intrinsics K
 * We provide the node's Id to get the correspondent Jacobian
 *
 *
 * z is a 2d line coordinates in the camera plane, in P^2
 * T is the transformation encoded by the 3D pose, the local frame (sensor)
 * p1, p2  are the 3d point encoding the point positions in the map coordinate frame
 *
 *
 */

class FactorCameraProj3dLine : public Factor
{
  public:
    FactorCameraProj3dLine(const Mat21 &obsPoint1, const Mat21 &obsPoint2,
            std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodePoint1,
            std::shared_ptr<Node> &nodePoint2,
            const Mat41 &camera_k,
            const Mat2 &obsInf = Mat2::Identity(),
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorCameraProj3dLine() override = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const;

    MatRefConst get_obs() const {return line_obs_;};
    VectRefConst get_residual() const {return r_;};
    MatRefConst get_information_matrix() const {return W_;};
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const {return J_;};

  protected:
    Mat31 line_obs_;
    Mat21 r_;
    Mat31 point1_, point2_, local_point1_, local_point2_;
    Mat41 camera_k_; // This encodes [fx, fy, cx, cy]
    SE3 Tinv_;
    Mat2 W_;//inverse of observation covariance (information matrix)
    Mat<2,12> J_;//Joint Jacobian: 6 (pose) + 3(point) + 3 (point) + 4 (K)

    /**
     * project_point in 3D to P^2 by the camera parameters in this class
    */
    Mat31 project_point_homog(const Mat31 &point);
    /**
     * calculate_image_line: given two points in the image coordiantes (pixels)
     * outputs the line in the plane that interesects both
    */
    Mat31 calculate_image_line(const Mat21 &p1, const Mat21 &p2);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTORCAMERAPROJ3DLINE_HPP_ */