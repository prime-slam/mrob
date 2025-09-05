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
 * nodePose3dVelTc.hpp
 *
 *  Created on: Sept 13, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODEPOSE3VELDTC_HPP_
#define NODEPOSE3VELDTC_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3tc.hpp" //requires including and linking SE3 library
#include "mrob/node.hpp"

namespace mrob{

class NodePose3dVelTc : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     * For 3D poses we use a transformation matrix 4x4
     * 
     * Note that the dimensionality of this node is 6, that is the DOF
     */
    NodePose3dVelTc(const Mat5 &initial_x, Node::nodeMode mode = STANDARD);
    /**
     * Initialization directly on SE3 a matrix
     */
    NodePose3dVelTc(const SE3tc &initial_x, Node::nodeMode mode = STANDARD);
    ~NodePose3dVelTc() = default;
    /**
     * Left update operation corresponds to
     * T'=exp(dxi^)*T, time of the update set to zero.
     * x'=vee(ln(T'))
     */
    void update(VectRefConst &dx) override;
    void update_from_auxiliary(VectRefConst &dx) override;
    void set_state(MatRefConst &x) override;
    void set_auxiliary_state(MatRefConst &x) override;
    MatRefConst get_state() const override {return state_.T();};
    MatRefConst get_auxiliary_state() const override {return auxiliaryState_.T();};
    void print() const override;
    Mat5 get_state_test() const {return state_.T();};
    void set_time_stamp(double t) override;

  protected:
    SE3tc state_;
    SE3tc auxiliaryState_; //an auxiliary vector for undoing updates


};


}


#endif /* NODEPOSE3D_HPP_ */
