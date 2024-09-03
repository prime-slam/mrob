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
#include "mrob/estimate_plane.hpp"
#include "mrob/SO3.hpp"
#include <fstream> 
#include <iostream>
#include <Eigen/Dense>

using namespace mrob;




FilterLidarInertial::FilterLidarInertial()
: Data_(nullptr), prev_state_stamp_(0.0)
{

}

FilterLidarInertial::~FilterLidarInertial()
{

}

void FilterLidarInertial::read_observations(const InertialData * LaodingData)
{
  Data_ = LaodingData;
  // fill in the function and change the heather.
}

void  FilterLidarInertial::print_data(){
  std::ofstream outfile("propagate_data.txt", std::ios::app); 

    if (!outfile.is_open()) {
        return;
    }
    outfile << "Timestamp: " << prev_state_stamp_ << std::endl;
    outfile << "Velocity: " << curr_state_.v().transpose() << std::endl;
    outfile << "Translation: " << curr_state_.t().transpose() << std::endl;
    outfile << "Rotation:" << std::endl << curr_state_.R() << std::endl;
    outfile << std::endl;
    outfile.close();


}
Mat3 FilterLidarInertial::calculatA(const Mat31 & u){
  //note that u here = omega*dt
  Mat3 A;
  Mat3 skew = hat3(u);
  double norm_u = u.norm();
  double alpha_u = (norm_u / 2.0) * (1.0 / tan(norm_u / 2.0));
  A = Mat3::Identity() - 0.5 * skew + ((1-alpha_u)/(norm_u * norm_u))*skew*skew; 
  return A.inverse(); 
}

Mat<18,18> FilterLidarInertial::calculateF(const Mat31 & omega, const Mat31 & a, const Mat3 & R, const double & dt){

  Mat<18,18> Fx = Mat<18,18>::Identity(); 
  Mat3 expW = SO3(-omega*dt).R(); 
  Mat3 A = calculatA(omega*dt); 
  Mat3 skew_a = hat3(a); 

  Fx.block<3,3>(0,0) = expW; 
  Fx.block<3,3>(0,3) = Mat3::Zero(); 
  Fx.block<3,3>(0,6) = Mat3::Zero(); 
  Fx.block<3,3>(0,9) = -A.transpose() * dt;
  Fx.block<3,3>(0,12) = Mat3::Zero(); 
  Fx.block<3,3>(0,15) = Mat3::Zero();
  // -----
  Fx.block<3,3>(3,0) = Mat3::Zero();
  Fx.block<3,3>(3,3) = Mat3::Identity(); 
  Fx.block<3,3>(3,6) = Mat3::Identity() * dt; 
  Fx.block<3,3>(3,9) = Mat3::Zero();
  Fx.block<3,3>(3,12) = Mat3::Zero(); 
  Fx.block<3,3>(3,15) = Mat3::Zero();
  // ---
  Fx.block<3,3>(6,0) = - R * skew_a * dt;
  Fx.block<3,3>(6,3) = Mat3::Zero(); 
  Fx.block<3,3>(6,6) = Mat3::Identity(); 
  Fx.block<3,3>(6,9) = Mat3::Zero();
  Fx.block<3,3>(6,12) = - R * dt; 
  Fx.block<3,3>(6,15) = Mat3::Identity() * dt;
  // -- 
  // Fx.block<3,3>(9,0) = Mat3::Zero();
  // Fx.block<3,3>(9,3) = Mat3::Zero(); 
  // Fx.block<3,3>(9,6) = Mat3::Zero(); 
  // Fx.block<3,3>(9,9) = Mat3::Identity();
  // Fx.block<3,3>(9,12) = Mat3::Zero(); 
  // Fx.block<3,3>(9,15) = Mat3::Zero();
  // //---
  // Fx.block<3,3>(12,0) = Mat3::Zero();
  // Fx.block<3,3>(12,3) = Mat3::Zero(); 
  // Fx.block<3,3>(12,6) = Mat3::Zero(); 
  // Fx.block<3,3>(12,9) = Mat3::Zero();
  // Fx.block<3,3>(12,12) = Mat3::Identity();
  // Fx.block<3,3>(12,15) = Mat3::Zero();
  // //--
  // Fx.block<3,3>(15,0) = Mat3::Zero();
  // Fx.block<3,3>(15,3) = Mat3::Zero(); 
  // Fx.block<3,3>(15,6) = Mat3::Zero(); 
  // Fx.block<3,3>(15,9) = Mat3::Zero();
  // Fx.block<3,3>(15,12) = Mat3::Zero();
  // Fx.block<3,3>(15,15) = Mat3::Identity();
return Fx; 
}

Mat<18,12> FilterLidarInertial::calculateG(const Mat31 & omega, const Mat31 & a, const Mat3 & R, const double & dt){
  Mat<18,12> G = Mat<18,12>::Zero(); 
  Mat3 A = calculatA(omega*dt);
  G.block<3,3>(0,0) = -A.transpose() * dt; 
  G.block<3,3>(6,3) = R * dt; 
  G.block<3,3>(9,6) = Mat3::Identity() *dt; 
  G.block<3,3>(12,9) = Mat3::Identity() *dt; 
  return G;
  }
void  FilterLidarInertial::propagate(){
  //  if (init) {
  //       if (Data && !Data->imu_buffer.empty()) {
  //           prev_state_stamp = Data->imu_buffer.front().stamp;
  //           prev_a =  Data->imu_buffer.front().linear_acceleration;
  //           Global_frame = mrob::SO3(mrob::quat_to_so3( Data->imu_buffer.front().orientation));
  //           init = false;
  //       } else {
  //           return;
  //       }
  //   }
  if (Data_ && !Data_->imu_buffer.empty()) {   
    for (const auto &imu_msg : Data_->imu_buffer) {
      if (init_) {
              prev_state_stamp_ = imu_msg.stamp;
              prev_a_ =  imu_msg.linear_acceleration;
              Global_frame_ = mrob::SO3(mrob::quat_to_so3( imu_msg.orientation));
              gravity_ = estimate_gravity_vector(Data_->imu_buffer);
              std::cout<<"Gravity "<<gravity_<<std::endl;
              init_ = false;

      }else{
        double dt = imu_msg.stamp - prev_state_stamp_; 
        if (dt <= 0) continue;
        Mat31 a = ( prev_state_.inv().R() * Global_frame_.inv().R() * imu_msg.linear_acceleration) - ab_ + gravity_; 
        Mat31 v = prev_state_.v() + prev_a_ * dt; //transform it 
        Mat31 t = prev_state_.t()+ prev_state_.v() * dt ; //add acc:  0.5*prev_a_*dt*dt;
        if (imu_msg.angular_velocity.size() != 3) continue;
        Mat31 omega;
        omega = imu_msg.angular_velocity - gb_ ; 
        SO3 dR(prev_state_.R());
        dR.update_rhs(omega*dt);
        prev_state_.update(mrob::SE3vel(dR, t, v)); 
        prev_state_stamp_ = imu_msg.stamp; 
        prev_a_ = a;
      }
    } 

  }else {
    return;
  }
  curr_state_.update(prev_state_);
  print_data();
}
void  FilterLidarInertial::solve(){
}
void FilterLidarInertial::set_extrinsics(const mrob::Mat3 R, const mrob::Mat31 T){
this->lidar_to_imu_ = mrob::SE3(R,T); 
}

Mat31 FilterLidarInertial::estimate_gravity_vector(const std::vector<Imu_msg>& imu_buffer){
  const double alpha = 0.98;
  
  if (imu_buffer.empty()) {
        std::cerr << "IMU buffer is empty!" << std::endl;
        return;
    }
  Mat3 Globalframe = mrob::quat_to_so3( imu_buffer[0].orientation);
  Mat31 gravity = Globalframe * Mat31(0, 0, 1);
  for (size_t i = 1; i < imu_buffer.size(); ++i) {
        const Imu_msg& imu_msg_prev = imu_buffer[i - 1];
        const Imu_msg& imu_msg = imu_buffer[i];
        double dt = imu_msg.stamp - imu_msg_prev.stamp;
        Mat31 accel = Globalframe * imu_msg.linear_acceleration;
        accel.normalize();
        gravity = alpha * (gravity + dt * imu_msg.angular_velocity.cross(gravity)) + (1.0 - alpha) * accel;
        gravity.normalize();
        gravity *= -9.81;
    }

    return gravity;
}
void FilterLidarInertial::solve(){

  mrob::FGraphSolve graph(mrob::FGraphSolve::ADJ);
  // TODO 
  //! creat a state <3x7> 
  //! update node to NodeInertial3d 
  //! update get_state
  //! update jacobians 

  std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(mrob::SE3(prev_state_.T()), mrob::Node::ANCHOR));
  mrob::factor_id_t id0 = graph.add_node(n0);
  std::shared_ptr<mrob::Node> n1(new mrob::NodePose3d(mrob::SE3(curr_state_.T())));
  mrob::factor_id_t id = graph.add_node(n1);
  for (size_t i = 1; i < Data_->pointcloud.size(); ++i) {
    auto nn = Data_->NN[i]; 
    auto n5n = Data_->neighbours[i];
    mrob::LidarPoint lidar_point = Data_->pointcloud[i]; 
    int N =n5n.size(); 
    Eigen::MatrixXd reshaped_matrix(N, 3);
    for(int j = 0; j < N; ++j) {
      reshaped_matrix.row(j) = n5n[j].point.transpose();
    }
    Mat31 normal = mrob::estimate_normal(reshaped_matrix);
    double time = lidar_point.stamp - Data_->lidar_begin_time; 
    double weight = 1; 
    mrob::Mat1 obsInf; 
    std::shared_ptr<mrob::Factor> factor = std::make_shared<mrob::Factor1Pose3d1AnchorInterpolated>(lidar_point.point, nn.point, normal, weight, n0, n1, time, obsInf); 
    graph.add_factor(factor);
  }
  graph.print(true);
  graph.solve();
  graph.print(true);
  std::cout << "\n\n\nSolved, chi2 = " << graph.chi2() << std::endl;
  auto node =  graph.get_node(id); 
  //mrob::Mat4 st = node.get_state(); 
  //?velocity ???? 
  //TODO:
  //* Publish transform
  //* add transformed points to Map  

  
}


Mat61 ComputeTdelta(SE3 Ta, SE3 Tb, double time)
{
    Mat61 xi_delta = (Tb * Ta.inv()).ln_vee() * time;
    return xi_delta;
}

SE3 InterpolatePose(SE3 Ta, SE3 Tb, double time)
{   
    Mat61 xi_delta = ComputeTdelta(Ta, Tb, time);
    SE3 Tt = SE3(xi_delta) * Ta;
    return Tt;
}

Factor1Pose3d1AnchorInterpolated::Factor1Pose3d1AnchorInterpolated(const Mat31 &observation_point, 
                                                                        const Mat31 &map_point, 
                                                                        const Mat31 &normal, 
                                                                        const double &weight,
                                                                        std::shared_ptr<Node> &begin_node, 
                                                                        std::shared_ptr<Node> &end_node, 
                                                                        const double &time,
                                                                        const Mat1 &obsInf, 
                                                                          Factor::robustFactorType robust_type):
                                                                          Factor(1,12, robust_type), 
                          observation_point_(observation_point), 
                          map_point_(map_point), 
                          time_(time),
                          Tx_(Mat31::Zero()), 
                          normal_(normal), 
                          weight_(weight), 
                          r_(0.0), 
                          W_(obsInf)
{
    neighbourNodes_.push_back(begin_node);
    neighbourNodes_.push_back(end_node);
    id_begin_node = 0;
    id_end_node = 1;
}
void Factor1Pose3d1AnchorInterpolated::evaluate_residuals()
{
    // r = <pi, Tp>
    Mat4 begin_T = get_neighbour_nodes()->at(id_begin_node)->get_state();
    Mat4 end_T = get_neighbour_nodes()->at(id_end_node)->get_state();
    SE3 T_taw = InterpolatePose(SE3(begin_T), SE3(end_T), time_);
    Tx_ = T_taw.transform(observation_point_);
    r_ = weight_ * Mat1(normal_.dot(Tx_ - map_point_));
}

void Factor1Pose3d1AnchorInterpolated::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat4 begin_T = get_neighbour_nodes()->at(id_begin_node)->get_state();
    Mat4 end_T = get_neighbour_nodes()->at(id_end_node)->get_state();
    Mat61 xi_delta = ComputeTdelta(SE3(begin_T), SE3(end_T), time_);
    Mat6 J_begin_T = (1 - time_) * SE3(xi_delta).adj(); // should be "I" as its anchor &G/&T_o = 0
    Mat6 J_end_T = time_ * Mat6::Identity(); // should change to J_begin_T
    Mat<6,12> Jxi;
    Jxi << J_begin_T, J_end_T;
    Mat<3,6> Jr;
    Jr << -hat3(Tx_) , Mat3::Identity();
    J_ = weight_ * normal_.transpose() * Jr * Jxi;
}
void Factor1Pose3d1AnchorInterpolated::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor1Pose3d1AnchorInterpolated::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs point x= \n" << observation_point_
              << "\nobs point y =\n" << map_point_
              << "\nobs normal y =\n" << normal_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}