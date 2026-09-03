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
 * nodeSim3.hpp
 *
 *  Created on: September 1, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODESIM3_HPP_
#define NODESIM3_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/Sim3.hpp" //requires including and linking SE3 library
#include "mrob/node.hpp"

namespace mrob{

class NodeSim3 : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     * For 3D poses we use a transformation matrix 4x4
     * 
     * Note that the dimensionality of this node is 6, that is the DOF
     */
    NodeSim3(const Mat4 &initial_x, Node::nodeMode mode = STANDARD);
    /**
     * Initialization directly on SE3 a matrix
     */
    NodeSim3(const Sim3 &initial_x, Node::nodeMode mode = STANDARD);
    ~NodeSim3() = default;
    /**
     * Left update operation corresponds to
     * T'=exp(dxi^)*T
     * x'=vee(ln(T'))
     */
    void update(VectRefConst &dx) override;
    void update_from_auxiliary(VectRefConst &dx) override;
    void set_state(MatRefConst &x) override;
    void set_auxiliary_state(MatRefConst &x) override;
    MatRefConst get_state() const override {return state_.T();};
    MatRefConst get_auxiliary_state() const override {return auxiliaryState_.T();};
    void print() const override;

  protected:
    Sim3 state_;
    Sim3 auxiliaryState_; //an auxiliary vector for undoing updates


};


}


#endif /* NodeSim3_HPP_ */
