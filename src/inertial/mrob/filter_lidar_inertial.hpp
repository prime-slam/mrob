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


#include "/home/mars_ugv/mrob/src/common/mrob/matrix_base.hpp"


namespace mrob
{
struct Orientation {
    double x;
    double y;
    double z;
    double w;
}; 
struct Angular_vel {
    double x;
    double y;
    double z;
};

struct Linear_acc {
    double x;
    double y;
    double z;
};
struct Imu_msg {
    double stamp;
    Orientation orientation;
    Angular_vel angular_velocity;
    Linear_acc linear_acceleration;

    Mat3 orientation_covariance;
    Mat3 angular_velocity_covariance;
    Mat3 linear_acceleration_covariance;
};

struct LidarPoint
{
  LidarPoint() // constructor 
  {
    point(0,0) = 0.0;
    point(1,0) = 0.0;
    point(2,0) = 0.0; 
    stamp = 0.0; 
    intensity = 0.0 ;
  };
  Mat31 point; 
  float stamp;  // time in nanosec 
  float intensity;
};

struct InertialData
{
  InertialData()
  {
    
  };
  std::vector<Imu_msg> imu_buffer; 
  std::vector<LidarPoint> pointcloud;
};
class FilterLidarInertial
{
  private: 
    InertialData Data;
  public:
    // Here constructor shou,d have all covariances values and hyperparams.
    FilterLidarInertial();
    ~FilterLidarInertial();
    void read_observations(const InertialData *LaodingData);
    void print_data();
    void propagate();
    void solve();
    void init_state();

};

}//namespace



#endif /* FILTERLIDARINERTIAL_HPP_ */