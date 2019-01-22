/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * gicp.hpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef GICP_HPP_
#define GICP_HPP_

#include "mrob/base_transf.hpp"

/**
 * Custom implementation of the GICP using SE3 optimization (improvement over Euler rotations)
 *
 * This method calculates the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = R*xh + t
 *
 *
 * T = min sum || y - Tx ||S
 *
 * The covariances provided are of the form S = R diag(e,1,1) R', so they MUST have been already processed.
 * The right way is a block matrix of covariances of the form Cov = [Cov_1, Cov_2,..., Cov_N], i.e, Cov \in R^{3x3N}
 */


namespace mrob{

class Gicp:  public BaseTransf {
  public:
    Gicp(const Eigen::Ref<const MatX> X, const Eigen::Ref<const MatX> Y,
            const Eigen::Ref<const MatX> covX, const Eigen::Ref<const MatX> covY);
    virtual ~Gicp();
    virtual int solve();

  protected:
    const MatX covX_, covY_;
};

}//end namespace


#endif /* Gicp_HPP_ */
