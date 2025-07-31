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
 * nodeGravity3d.hpp
 *
 *  Created on: July 31, 2025
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODEGRAVITY3D_HPP_
#define NODEGRAVITY3D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{

class NodeGravity3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodeGravity3d(const Mat31 &initial_x, Node::nodeMode mode = STANDARD);
    //NodePose3d(const SE3 &initial_T);
    ~NodeGravity3d() = default;

    void update(VectRefConst &dx) override;
    void update_from_auxiliary(VectRefConst &dx) override;
    void set_state(MatRefConst &x) override;
    void set_auxiliary_state(MatRefConst &x) override;
    MatRefConst get_state() const override {return state_;}
    MatRefConst get_auxiliary_state() const override {return auxiliaryState_;}
    void print() const override;

  protected:
    Mat31 state_;
    Mat31 auxiliaryState_;

    //new function to ensure the gravity vector stays in the magnitude 9.8
    void regenerate(void);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NodeGravity3d_HPP_ */
