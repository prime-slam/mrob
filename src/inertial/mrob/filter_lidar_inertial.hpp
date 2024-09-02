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
 * filter_lidar_inertial.hpp
 *
 *  Created on: Feburary 1, 2024
 *      Author: Ahmed Baza
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef FILTERLIDARINERTIAL_HPP_
#define FILTERLIDARINERTIAL_HPP_


// #include "mrob/matrix_base.hpp"
#include <mrob/matrix_base.hpp>
#include <mrob/SE3velCov.hpp>
#include <mrob/SO3.hpp>
#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/factor2Poses3d.hpp"
#include "mrob/factors/factor2PosesPoint2PlaneInterpolated.hpp"
namespace mrob
{

struct Imu_msg {
    double stamp;
    Mat41 orientation;
    Mat31 angular_velocity;
    Mat31 linear_acceleration;
    Mat3 orientation_covariance;
    Mat3 angular_velocity_covariance;
    Mat3 linear_acceleration_covariance;
};

struct LidarPoint
{
  LidarPoint() : point(Mat31::Zero()), stamp(0.0), intensity(0.0)// constructor 
  {
    /*point(0,0) = 0.0;
    point(1,0) = 0.0;
    point(2,0) = 0.0; 
    stamp = 0.0; 
    intensity = 0.0 ;*/
  };
  Mat31 point; 
  float stamp;  // //time in nanosec 
  float intensity;
};
struct NeighbourPoint{
NeighbourPoint() : point(Mat31::Zero())// // constructor 
  {

  };
  Mat31 point; 
};

struct InertialData
{
  InertialData()
  {
    
  };
  double lidar_begin_time; // connect 
  double lidar_end_time; // connect 
  std::vector<Imu_msg> imu_buffer; // done 
  std::vector<LidarPoint> pointcloud; // done 
  std::vector<NeighbourPoint> NN;//connect 
  std::vector<std::vector<NeighbourPoint>> neighbours; //connect 
};
class FilterLidarInertial
{
  public:
    // // Here constructor shou,d have all covariances values and hyperparams.
    // // TODO trail undrscore all class variables 
    FilterLidarInertial();
    ~FilterLidarInertial();
    void read_observations(const InertialData *LaodingData);
    void print_data();
    void propagate();
    void solve();
    void evaluate_residuals();
    void evaluate_jacobians();
    void evaluate_chi2();
    void set_extrinsics(const mrob::Mat3 R, const mrob::Mat31 T); 
    Mat31 estimate_gravity_vector(const std::vector<Imu_msg>& imu_buffer); 
    mrob::SE3 lidar_to_imu_; 
    mrob::SO3 Global_frame_;
    mrob::SE3vel prev_state_;
    mrob::SE3vel curr_state_;
    Mat31 gravity_ = Mat31(0, 0, 9.81);
    bool init_ = true; 
    double prev_state_stamp_ = 0; 
    mrob::Mat31 prev_a_;
    mrob::Mat31 ab_; 
    mrob::Mat31 gb_; 
  private: 
    const InertialData *Data_;
    
};

class Factor1Pose3d1AnchorInterpolated : public Factor{


public:
    Factor1Pose3d1AnchorInterpolated(const Mat31 &observation_point, 
                                        const Mat31 &map_point, 
                                        const Mat31 &normal, 
                                        const double &weight,
                                        std::shared_ptr<Node> &anchor_node, 
                                        std::shared_ptr<Node> &target_node, 
                                        const double &time,
                                        const Mat1 &obsInf,
                                        Factor::robustFactorType robust_type = Factor::robustFactorType::CAUCHY);
    ~Factor1Pose3d1AnchorInterpolated() override = default;
    virtual void evaluate_residuals() override;
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;
    virtual void print() const;
    MatRefConst get_obs() const override {return r_;};
    VectRefConst get_residual() const override {return r_;};
    MatRefConst get_information_matrix() const override {return W_;};
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const override {return J_;};

  protected:
    Mat31 observation_point_, map_point_,  Tx_;
    Mat31 normal_;
    double time_;
    double weight_;
    double id_begin_node;
    double id_end_node;
    // the residual of the point projected to the plane:
    Mat1 r_;//residual, for convention, we will keep an eigen object for the scalar
    Mat1 W_;
    Mat<1,12> J_;
};


}//namespace



#endif /* FILTERLIDARINERTIAL_HPP_ */