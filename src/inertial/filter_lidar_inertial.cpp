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
 * filter_lidar_inertial.cpp
 *
 *  Created on: Feburary 1, 2024
 *      Author: Ahmed Baza
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include "mrob/filter_lidar_inertial.hpp"
#include <fstream> 

using namespace mrob;
FilterLidarInertial::FilterLidarInertial()
: Data(nullptr), prev_state_stamp(0.0)
{

}

FilterLidarInertial::~FilterLidarInertial()
{

}

void FilterLidarInertial::read_observations(const InertialData * LaodingData)
{
  Data = LaodingData;
  // fill in the function and change the heather.
}

void  FilterLidarInertial::print_data(){
  std::ofstream outfile("propagate_data.txt", std::ios::app); 

    if (!outfile.is_open()) {
        return;
    }
    outfile << "Timestamp: " << prev_state_stamp << std::endl;
    outfile << "Velocity: " << curr_state.v().transpose() << std::endl;
    outfile << "Translation: " << curr_state.t().transpose() << std::endl;
    outfile << "Rotation:" << std::endl << curr_state.R() << std::endl;
    outfile << std::endl;
    outfile.close();


}
void  FilterLidarInertial::propagate(){
   if (init) {
        if (Data && !Data->imu_buffer.empty()) {
            prev_state_stamp = Data->imu_buffer.front().stamp;
            init = false;
        } else {
            return;
        }
    }
  for (const auto &imu_msg : Data->imu_buffer) {
        double dt = imu_msg.stamp - prev_state_stamp; 
        if (dt <= 0) continue;
        Mat31 gravity(0, 0, -9.81);
        Mat31 a = imu_msg.linear_acceleration+gravity;//check reference, seems it needs rotation
        Mat31 v = prev_state.v() + a * dt;
        Mat31 t = prev_state.t()+ v * dt;
        if (imu_msg.angular_velocity.size() != 3) continue;
        Mat3 omega;
        // chek: hat3()
        omega << 0, -imu_msg.angular_velocity(2), imu_msg.angular_velocity(1),
        imu_msg.angular_velocity(2), 0, -imu_msg.angular_velocity(0),
        -imu_msg.angular_velocity(1), imu_msg.angular_velocity(0), 0 ;

        SO3 dR(prev_state.R());
        dR.update_rhs(omega*dt);
        //Mat3 dR = Mat3::Identity() + omega * dt + 0.5 * omega * omega * dt * dt;
        //SO3 orientation = prev_state.R() * dR.R();
        prev_state.update(mrob::SE3vel(dR, t, v)); 
        prev_state_stamp = imu_msg.stamp; 
  }
  curr_state.update(prev_state);
  print_data();
}
void  FilterLidarInertial::solve(){
}