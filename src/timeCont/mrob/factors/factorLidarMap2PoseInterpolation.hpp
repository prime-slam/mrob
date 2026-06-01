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
 * factorLidarMap2PoseInterpolation.hpp
 *
 *  Created on: Jul 17, 2025
 *      Author: Ahmed Baza
                Ahmed.Baza@skoltech.ru 
 *              Mobile Robotics Lab, Skoltech 
 */

 #ifndef FACTORLIDARMAP2POSEINTERPOLATION_HPP_
 #define FACTORLIDARMAP2POSEINTERPOLATION_HPP_
 
 
 #include "mrob/matrix_base.hpp"
 #include "mrob/SE3tc.hpp"
 #include "mrob/SE3.hpp"
 #include "mrob/factor.hpp"
 
 namespace mrob{
 
 /**
  * factor lidar map 2 pose interpolation creates a high dimension interpolation between poses
  * and matches it with lidar observations.
  */

class FactorLidarMap2PoseInterpolation : public Factor
{
public:
    FactorLidarMap2PoseInterpolation(const Mat41 &observation,const Mat31 &map_point, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat4 &offset_lidar_imu, const Mat3 &obsInf, 
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~FactorLidarMap2PoseInterpolation() = default;
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
    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J_;}
    
    
    protected:
    Mat41 obs_; // obs = [X, Y, Z, t] lidar observation
    Mat31 r_; //residual [TP - Z(:3)]
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat<3,18> J_;//Joint Jacobian
    SE3 T_taw_; // interpolated pose
    Mat31 map_point_, point_obs_imu_frame_; // map point
    SE3 T_offset_lidar_imu_; // offset from lidar to imu XXX: I think it is the other way, from IMU to LiDAR. Plase check
    matData_t thau_taw_; //Interpolation fraction. Keept for jacobian calculation
    Mat61 compute_delta(SE3 Ta, SE3 Tb, matData_t time); 
    void interpolate_pose(const SE3tc &T_origin, const SE3tc &T_target,const matData_t &t_obs); // interpolate pose between two poses 


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3DTWISTMATCHING_HPP_ */
