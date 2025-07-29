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
    FactorLidarMap2PoseInterpolation(const Mat51 &observation,const Mat31 &map_point, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, 
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
    Mat61 compute_delta(SE3 Ta, SE3 Tb, matData_t time); 
    void print() const override;
    void interpolate_pose(const SE3tc &T_origin, const SE3tc &T_target,const matData_t &t_obs); // interpolate pose between two poses 
    MatRefConst get_obs() const override {return obs_;}
    VectRefConst get_residual() const override {return r_;}
    MatRefConst get_information_matrix() const override {return W_;}
    MatRefConst get_jacobian(factor_id_t /*id */) const override {return J_;}
    

protected:
    Mat51 obs_; // obs = [X, Y, Z, 0,t] lidar observation
    Mat31 r_; //residual [TP - Z(:3)]
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat<3,9> J_;//Joint Jacobian
    SE3 T_taw_; // interpolated pose
    Mat31 map_point_; // map point
    


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3DTWISTMATCHING_HPP_ */
