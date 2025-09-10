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
 * nodeSO3.hpp
 *
 *  Created on: Jul 18, 2025
 *      Author: Ivan Kakurin
 *              i.kakurin@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

 #ifndef NODESO3_HPP_
 #define NODESO3_HPP_
 
 #include "mrob/matrix_base.hpp"
 #include "mrob/SO3.hpp" //requires including and linking SE3 library
 #include "mrob/node.hpp"
 
 namespace mrob{
 
 class NodeSO3 : public Node
 {
   public:
     /**
      * For initialization, requires an initial estimation of the state.
      * For 3D poses we use a matrix 3x3
      * 
      */
     NodeSO3(const Mat3 &initial_x, Node::nodeMode mode = STANDARD);
     /**
     * Initialization directly on SO3 a matrix
     */
     NodeSO3(const SO3 &initial_x, Node::nodeMode mode = STANDARD);
     ~NodeSO3() = default;
     /**
      * Left update operation corresponds to
      * R'=exp(dxi^)*T
      * x'=vee(ln(T'))
      */
     void update(VectRefConst &dx) override;
     void update_from_auxiliary(VectRefConst &dx) override;
     void set_state(MatRefConst &x) override;
     void set_auxiliary_state(MatRefConst &x) override;
     MatRefConst get_state() const override {return state_.R();};
     MatRefConst get_auxiliary_state() const override {return auxiliaryState_.R();};
     void print() const override;
 
   protected:
     SO3 state_;
     SO3 auxiliaryState_; //an auxiliary vector for undoing updates
 
 
 };
 
 
 }
 
 
 #endif /* NODEPOSE3D_HPP_ */
 