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
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef MROB_NODEPOSE2D_H
#define MROB_NODEPOSE2D_H

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{


    /**
     * This class NodePose2d for now is parametrized on XYT coordinates
     * TODO compare with SE(2)
     */
    class NodePose2d : public Node {
    public:
        /**
         * For initialization, requires an initial estimation of the state.
         */
        explicit NodePose2d(const Mat31 &initial_x, Node::nodeMode mode = STANDARD);
        ~NodePose2d() = default;

        void update(VectRefConst &dx) override;
        void update_from_auxiliary(VectRefConst &dx) override;
        void set_state(MatRefConst &x) override;
        void set_auxiliary_state(MatRefConst &x) override;
        MatRefConst get_state() const override {return state_;};
        MatRefConst get_auxiliary_state() const override {return auxiliaryState_;};
        void print() const override;
    protected:
        Mat31 state_;
        Mat31 auxiliaryState_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif //MROB_NODEPOSE2D_H
