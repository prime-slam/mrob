/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * planeRegistration.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PLANEREGISTRATION_HPP_
#define PLANEREGISTRATION_HPP_

#include <vector>
#include "mrob/SE3.hpp"
#include <Eigen/StdVector>
#include "mrob/plane.hpp"

#include <unordered_map>
#include <memory>


namespace mrob{

/**
 * class PlaneRegistration introduced a class for the alignment of
 * planes.
 */
class PlaneRegistration{
  public:
    PlaneRegistration();
    ~PlaneRegistration();

    int solve();
    std::vector<SE3>& get_transformations();//if solved

  protected:
    std::unordered_map<int, std::shared_ptr<Plane>> planes_;
    std::vector<SE3> transformations_;
    // flag for detecting when is has been solved
    uint_t isSolved_;
    uint_t time_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
