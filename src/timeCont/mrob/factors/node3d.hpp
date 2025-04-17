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
 *  Created on: April 17, 2025
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

 #ifndef NODE3D_HPP_
 #define NODE3D_HPP_
 
 #include "mrob/matrix_base.hpp"
 #include "mrob/node.hpp"
 
 namespace mrob{
 
 class Node3d : public Node
 {
   public:
     /**
      * For initialization, requires an initial estimation of the state.
      * 3D vector
      * 
      * Note that the dimensionality of this node is 6, that is the DOF
      */
     Node3d(const Mat31 &initial_x, Node::nodeMode mode = STANDARD);
     ~Node3d() = default;
     /**
      * Left update operation corresponds to
      * T'=exp(dxi^)*T, time of the update set to zero.
      * x'=vee(ln(T'))
      */
     void update(VectRefConst &dx) override;
     void update_from_auxiliary(VectRefConst &dx) override;
     void set_state(MatRefConst &x) override;
     void set_auxiliary_state(MatRefConst &x) override;
     MatRefConst get_state() const override {return state_;};
     MatRefConst get_auxiliary_state() const override {return auxiliaryState_;};
     void print() const override;
 
 
   protected:
     Mat31 state_;
     Mat31 auxiliaryState_; //an auxiliary vector for undoing updates
 
 
 };
 
 
 }
 
 
 #endif /* NODE3D_HPP_ */
 