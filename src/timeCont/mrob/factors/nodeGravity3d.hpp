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

/**
 * Orthonormal basis B (3x2) of the plane perpendicular to g, with [b1, b2, g/|g|] right handed.
 * The choice is deterministic in g, so that nodes and factors that linearise at the same
 * gravity vector use the same tangent coordinates.
 */
Mat<3,2> gravity_tangent_basis(const Mat31 &g);

/**
 * Gravity vector with fixed magnitude: a point on the sphere S^2 of radius |g| (2 DoF).
 *
 * The state is stored as the 3-vector g (world frame, "up" convention: specific force at rest = R^T g).
 * The magnitude is set at construction and never changes: updates are rotations of g.
 *
 * Update (retraction), for dx in R^2:
 *      g' = Exp( hat(B dx) ) g,       B = gravity_tangent_basis(g)  (3x2)
 * and therefore
 *      d g' / d dx = -hat(g) B                       (3x2)
 * which is the matrix factors must multiply their derivative w.r.t. the 3-vector g by.
 */
class NodeGravity3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state. Its norm is the (fixed)
     * magnitude of the gravity vector for the lifetime of the node.
     */
    NodeGravity3d(const Mat31 &initial_x, Node::nodeMode mode = STANDARD);
    ~NodeGravity3d() = default;

    void update(VectRefConst &dx) override;
    void update_from_auxiliary(VectRefConst &dx) override;
    void set_state(MatRefConst &x) override;
    void set_auxiliary_state(MatRefConst &x) override;
    MatRefConst get_state() const override {return state_;}
    MatRefConst get_auxiliary_state() const override {return auxiliaryState_;}
    void print() const override;

    matData_t get_magnitude() const {return magnitude_;}

  protected:
    Mat31 state_;
    Mat31 auxiliaryState_;
    matData_t magnitude_;

    // g <- Exp(hat(B(g) dx)) g, followed by a re-normalisation to remove numerical drift
    static Mat31 retract(const Mat31 &g, const Mat21 &dx, matData_t magnitude);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NodeGravity3d_HPP_ */
