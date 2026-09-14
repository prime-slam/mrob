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
 * Sim3.hpp
 *
 *  Created on: Aug 26, 2026
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef SIM3_HPP_
#define SIM3_HPP_

 
#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"
#include "mrob/SE3.hpp"
 
 
 
 /**
  *  \brief Sim(3) (group) in 3d
  *  Is the group representing rotations scale and translations:
  *  Sim3 = {T = [sR  t]  |  R \in SO3 , t \in Re^3 , s \in Re}
  *               [0  1]
  *  xi =[theta , pho, lambda] \in Re^7, where theta \in Re^3 represents the rotation
  *  and pho the translation and lamda is a scalar represting the scale .
  */
 namespace mrob{
 
 
 class Sim3
 {
 public:
     /**
      * Constructor, requires the Transformation matrix 4x4
      */
     Sim3(const Mat4 &T = Mat4::Identity() );
     /**
      * Constructor, requires the Lie algebra xi^ \in Sim3 representing the rigid body
      * transformation around the identity, by default generates T = exp(0^) = I
      */
     Sim3(const Mat71 &xi);
     /**
      * Constructor, requires the Transformation in Sim3
      */
     Sim3(const Sim3 &T);
     /**
      * This constructor allows to construct from Eigen expressions
      * Eigen suggestion: TopicCustomizingEigen.html
      */
     template<typename OtherDerived>
     Sim3(const Eigen::MatrixBase<OtherDerived>& rhs);
     /**
      * This method allows you to assign Eigen expressions to Sim3
      */
     Sim3& operator=(const Sim3& rhs);
 
     /**
      * This method allows you to Multiply Sim3 expressions
      */
     Sim3 operator*(const Sim3& rhs) const;
     /**
      * Multiplication function, same as above, mainly for python
      */
     Sim3 mul(const Sim3& rhs) const;
 
     /**
      * This is our *default* way to update transformations, from the Left hand side of T
      * Updates the current transformation with the incremental dxi \in Sim3
      * T'=exp(dxi^) * T
      */
     void update_lhs(const Mat71 &dxi);
     /**
      * Updates the current transformation with the incremental dxi \in Sim3
      * T'= T * exp(dxi^)
      */
     void update_rhs(const Mat71 &dxi);
     /**
      *  Exponential mapping of a skew symetric matrix in Sim3.
      */
     void exp(const Mat71 &xi);
     /**
      * Returns the vector xi \in R^7 which corresponds to the Lie algebra Sim3
      */
     Mat71 ln() const;
     /**
      * Transforms a point p = (x,y,z)' such as res = T*p.
      * This function saves to transform to homogeneous coordinates.
      */
     Mat31 transform(const Mat31 & p) const;
     /**
      * Transforms an array of points P = {p_n} = (x,y,z)'_n such as res = T*p_n.
      * The array is of the form Nx3 (usual convention from arrays)
      * This function saves to transform to homogeneous coordinates.
      */
     MatX transform_array(const MatX &P) const;
     /**
      * Inverse: T^-1 = [1/sR', -1/sR't]
      *                 [0      1]
      */
     Sim3 inv(void) const;
     /**
      * Adjoint: T Exp(x) = Exp ( Adj_T x) T
      *          Adj_T = [R ,   0,   0]
      *                  [t^R, sR,  -t]
      *                  [0     0    1]
      */
     Mat7 adj() const;
     /**
      * T method returns a matrix 4x4 of the Sim3 transformation. Ref<> is more convinient than
      * the matrix for the factor/nodes base class definitions and python bindings
      */
     //Mat4 T() const;
     const Eigen::Ref<const Mat4> T() const;
     /**
      * ref2T returns a non-const reference to the matrix T to modify its content directly
      */
     Mat4& ref2T();
     /**
      * R method returns a matrix 3x3 of the SO3 rotation corresponding to the subblock matrix
      */
     Mat3 sR() const;
     /**
      * t method returns translation
      */
     Mat31 t() const;
     /**
      * p method returns translation, equivalent to t()
      */
     Mat31 p() const;
     /**
      * s method returns scale
      */
     matData_t s() const;
     /**
      * Regenerate, does the following operation:
      * T = Exp ( Ln(T) )
      */
     void regenerate();
 
     Mat41 transform_plane(const Mat41 &pi);
 
 
     void print(void) const;
     void print_lie(void) const;
 
     /**
      * @brief Generates string representation of the object
      *
      * @return std::string object to print
      */
     std::string toString() const;
 
 protected:
     Mat4 T_;
 
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 
 
 /**
  * Hat operator xi^ = [sigm  -w3   w2 v1
  *                     w3   sigm  -w1 v2
  *                    -w2     w1  sigm  v3
  *                      0      0    0   0]
  */
 Mat4 hat7(const Mat71 &xi);
 /**
  * Vee operator (v), the inverse of hat
  */
 Mat71 vee7(const Mat4 &xi_hat);
 
 
 }// end namespace
 #endif /* Sim3_HPP_ */
 