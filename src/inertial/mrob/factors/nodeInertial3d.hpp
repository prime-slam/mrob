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
 * nodeInertial3d.hpp
 *
 *  Created on: January 11, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef NODEINERTIAL3D_HPP_
#define NODEINERTIAL3D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
//#include "mrob/SE3_extended.hpp"
#include "mrob/node.hpp"

namespace mrob{

/**
 * The nodeInertial3d is a vertex representing the required state variables
 * for the interial manifold. Thse include:
 *  - extended pose SE(3): R, t, v
 *  - gyro bias and acc bias
 *
 * This factor will just update the value of **target node** as T = T_origin * integration(w(t))
 * 
 * This node does not provide Jacobian or returns a zero error (chi2) to dnote it does not contribute to
 * the optimization process
 *
 *
 *
 */

class NodeInertial3d : public Node
{
  public:
    NodeInertial3d(const Mat<3,7> state,
            Node::nodeMode mode = STANDARD);
    ~NodeInertial3d() = default;

    void update(VectRefConst &dx) override;
    void update_from_auxiliary(VectRefConst &dx) override;
    void set_state(MatRefConst &x) override;
    void set_auxiliary_state(MatRefConst &x) override;
    MatRefConst get_state() const override {return state_;};
    MatRefConst get_auxiliary_state() const override {return auxiliary_state_;};
    void print() const override;

  protected:
    // joint state including s = [rotation(3x3), position, velocity, gyrro_bias, acc_bias]
    Mat<3,7> state_;
    Mat<3,7> auxiliary_state_;
    //SE3extened pose;//TODO needs to approve first the extended SE3 MR
    Mat31 gyro_bias_, acc_bias_;


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


/**
 * Propagation function.
 * 
 * Input:
 *  - node interial prior IC
 *  - obs from acc and gyro (TBD)
*/

// void propagate_interial3d(NodeInertial3d node, const Mat91 &obs);

}




#endif /* NODEINERTIAL3D_HPP_ */