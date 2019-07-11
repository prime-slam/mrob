/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * factor2Poses3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3d.hpp"

#include <iostream>
#include <Eigen/Cholesky>

using namespace mrob;


Factor2Poses3d::Factor2Poses3d(const Mat61 &observation, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget):
        Factor(6,12), obs_(observation), Tobs_(observation), W_(obsInf)
{
    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);

        // inverse observations to correctly modify this
        obs_ = -observation;
        Tobs_ = SE3(obs_);
    }
    WT2_ = W_.llt().matrixU();
    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // NOTE: carefull on the reference frame that Tobs is expressed on the X_origin frame, hence this change:
        // T_xo * (xo reference)T_obs = (global)T_obs * T_xo.
        // We could also use the adjoint to refer the manifold coordinates obs on the xo to the global frame (identity)
        Mat4 TxOrigin = nodeOrigin->get_stateT();
        SE3 T = SE3(TxOrigin) * Tobs_;//TODO how many SE3 structures are created here?
        nodeTarget->set_state(T.ln_vee());
    }
}

Factor2Poses3d::~Factor2Poses3d()
{
}

void Factor2Poses3d::evaluate_residuals()
{
    // r = h(x_O,x_T) - z (in general). From Origin we observe Target
    // Tr = Txo * (xo frame)Tobs * Txt^-1
    // NOTE: carefull on the reference frame that Tobs is expressed on the X_origin frame, hence this change:
    // T_xo * (xo reference)T_obs = (global)T_obs * T_xo.
    // We could also use the adjoint to refer the manifold coordinates obs on the xo to the global frame (identity)
    Mat4 TxOrigin = get_neighbour_nodes()->at(0)->get_stateT();
    Mat4 TxTarget = get_neighbour_nodes()->at(1)->get_stateT();
    Tr_ = SE3(TxOrigin) * Tobs_ * SE3(TxTarget).inv();
    r_ = Tr_.ln_vee();

    // XXX debuging, when upodates are too large
    if (0)// get_neighbour_nodes()->at(1)->get_id() == 629 )
    {
        std::cout << "incorrect update at factor "<< this->id_ << "residual = " << r_ << "Between nodes :" << std::endl;
        this->print();
        get_neighbour_nodes()->at(0)->print();
        get_neighbour_nodes()->at(1)->print();
    }
}
void Factor2Poses3d::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    J_.topLeftCorner<6,6>() = Mat6::Identity();
    J_.topRightCorner<6,6>() = -Tr_.adj();
}

void Factor2Poses3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

