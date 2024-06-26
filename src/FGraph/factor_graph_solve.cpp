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
 * factor_graph_solve.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/factor_graph_solve.hpp"
//#include "mrob/CustomCholesky.hpp"

#include <iostream>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>


using namespace mrob;
using namespace std;
using namespace Eigen;


FGraphSolve::FGraphSolve(matrixMethod method):
	FGraph(), matrixMethod_(method), optimMethod_(LM), N_(0), M_(0),
	lambda_(1e-6), solutionTolerance_(1e-2), buildAdjacencyFlag_(false),
    verbose_(false)
{

}

FGraphSolve::~FGraphSolve() = default;


uint_t FGraphSolve::solve(optimMethod method, uint_t maxIters,
        matData_t lambdaParam, matData_t solutionTolerance, bool verbose)
{
    /**
     * 2800 2D nodes on M3500
     * Time profile :13.902 % build Adjacency matrix,
     *               34.344 % build Information,
     *               48.0506 % build Cholesky,
     *               2.3075 % solve forward and back substitution,
     *               1.3959 % update values,
     *
     */
    lambda_ = lambdaParam;
    solutionTolerance_ = solutionTolerance;
    verbose_ = verbose;
    time_profiles_.reset();
    optimMethod_ = method;

    assert(stateDim_ > 0 && "FGraphSolve::solve: empty node state");

    uint_t iters(0);

    // Optimization
    switch(method)
    {
      case GN:
        this->optimize_gauss_newton();// false => lambda = 0
        this->update_nodes();
        iters = 1;
        break;
      case LM:
      case LM_ELLIPS:
        iters = this->optimize_levenberg_marquardt(maxIters);
        break;
      default:
        assert(0 && "FGraphSolve:: optimization method unknown");
    }



    // TODO add variable verbose to output times
    if (verbose_)
        time_profiles_.print();
        
    return iters; 
}

void FGraphSolve::build_problem(bool useLambda)
{

    // 1) Adjacency matrix A, it has to
    //    linearize and calculate the Jacobians and required matrices
    time_profiles_.start();
    this->build_adjacency();
    time_profiles_.stop("Adjacency");

    if (eigen_factors_.size()>0)
    {
        time_profiles_.start();
        this->build_info_EF();
        time_profiles_.stop("EFs Jacobian and Hessian");
    }

    // 1.2) builds specifically the information
    switch(matrixMethod_)
    {
      case ADJ:
        time_profiles_.start();
        this->build_info_adjacency();
        time_profiles_.stop("Info Adjacency");
        break;
      case SCHUR:
      default:
        assert(0 && "FGraphSolve: method not implemented");
    }

    // 1.3) (Optional) Eigen Factors

    // Structure for LM and dampening GN-based methods
    if (useLambda)
    {
        diagL_ = L_.diagonal();
    }
}

void FGraphSolve::optimize_gauss_newton(bool useLambda)
{
    // requires a Column-storage matrix
    SimplicialLDLT<SMatCol,Lower, AMDOrdering<SMatCol::StorageIndex>> cholesky;

    this->build_problem(useLambda);

    // compute cholesky solution
    time_profiles_.start();
    if (useLambda)
    {
        for (uint_t n = 0 ; n < N_; ++n)
        {
            if (optimMethod_ == LM)
                L_.coeffRef(n,n) = lambda_ + diagL_(n);//Spherical
            if (optimMethod_ == LM_ELLIPS)
                L_.coeffRef(n,n) = (1.0 + lambda_)*diagL_(n);//Elipsoid
        }
    }
    cholesky.compute(L_);
    time_profiles_.stop("Gauss Newton create Cholesky");
    time_profiles_.start();
    dx_ = cholesky.solve(b_);
    time_profiles_.stop("Gauss Newton solve Cholesky");

}

uint_t FGraphSolve::optimize_levenberg_marquardt(uint_t maxIters)
{
    //SimplicialLDLT<SMatCol,Lower, AMDOrdering<SMatCol::StorageIndex>> cholesky;


    // LM trust region as described in Bertsekas (p.105)

    // 0) parameter initialization
    //lambda_ = 1e-5;
    // sigma reference to the fidelity of the model at the proposed solution \in [0,1]
    matData_t sigma1(0.25), sigma2(0.8);// 0 < sigma1 < sigma2 < 1
    matData_t beta1(2.0), beta2(0.25); // lambda updates multiplier values, beta1 > 1 > beta2 >0
    //matData_t lambdaMax, lambdaMin; // XXX lower bound unnecessary

    matData_t currentChi2, deltaChi2, modelFidelity;
    uint_t iter = 0;

    do{
        iter++;
        // 1) solve subproblem and current error
        this->optimize_gauss_newton(true);// Test if solved anything? no nans
        currentChi2 = this->chi2(false);// TODO residuals don't need to be calculated again (see optimizer.cpp)
        this->synchronize_nodes_auxiliary_state();// book-keeps states to undo updates
        this->update_nodes();


        // 1.2) Check for convergence, needs update and re-evaluaiton of errors
        deltaChi2 = currentChi2 - this->chi2(true);
        // For now we enable diable this by an if
        // TODO later it should be on pre-processor as an option on the cmake
        if (verbose_)
        {
            std::cout << "\nFGraphSolve::optimize_levenberg_marquardt: iteration "
                    << iter << " lambda = " << lambda_ << ", error " << currentChi2
                    << ", and delta = " << deltaChi2
                    << std::endl;
        }
        if (deltaChi2 < 0)
        {
            // proposed dx did not improve, repeat 1) and reduce area of optimization = increase lambda
            lambda_ *= beta1;
            this->synchronize_nodes_state();
            continue;
        }

        // 1.3) check for convergence
        if (deltaChi2/currentChi2 < solutionTolerance_) //in ratio
        {
            if (verbose_)
                std::cout << "\nFGraphSolve::optimize_levenberg_marquardt: Converged Successfully" << std::endl;
            return iter;
        }


        // 2) Fidelity of the quadratized model vs non-linear chi2 evaluation.
        // f = chi2(x_k) - chi2(x_k + dx)
        //     chi2(x_k) - m_k(dx)
        // where m_k is the quadratized model = ||r||^2 - dx'*J' r + 0.5 dx'(J'J + lambda*D2)dx
        //modelFidelity = deltaChi2 / (dx_.dot(b_) - 0.5*dx_.dot(L_* dx_));
        // Update, according to H.B. Nielson, Damping Parameter In Marquardt’s Method, Technical Report IMM-REP-1999-05, Dept. of Mathematical Modeling, Technical University Denmark
        // (J'J ) dx = J'r , so this is substituted.
        modelFidelity = 2.0 * deltaChi2 / (dx_.dot(b_) - dx_.dot(lambda_ * diagL_* dx_));
        if (verbose_)
        {
            std::cout << "model fidelity = " << modelFidelity << " and m_k = " << dx_.dot(b_) << std::endl;
        }

        //3) update lambda
        if (modelFidelity < sigma1)
            lambda_ *= beta1;
        if (modelFidelity > sigma2)
            lambda_ *= beta2;


    } while (iter < maxIters);

    // output
    if (verbose_)
    {
        std::cout << "FGraphSolve::optimize_levenberg_marquardt: failed to converge after "
                << iter << " iterations and error " << currentChi2
                << ", and delta = " << deltaChi2
                << std::endl;
    }
    return 0; //Failed to converge is indicated with 0 iterations

}

void FGraphSolve::build_index_nodes_matrix()
{
    N_ = 0;
    for (size_t i = 0; i < active_nodes_.size(); ++i)
    {
        // calculate the indices to access
        uint_t dim = active_nodes_[i]->get_dim();
        factor_id_t id = active_nodes_[i]->get_id();
        indNodesMatrix_.emplace(id,N_);
        N_ += dim;
    }
}

void FGraphSolve::build_adjacency()
{
    // 1) Node indexes bookkept. We use a map to ensure the index from nodes to the current active_node
    indNodesMatrix_.clear();
    this->build_index_nodes_matrix();
    assert(N_ == stateDim_ && "FGraphSolve::buildAdjacency: State Dimensions are not coincident\n");


    // 2.1) Check for consistency. With 0 observations the problem does not need to be build, EF may still build it
    if (obsDim_ == 0)
    {
        buildAdjacencyFlag_ = false;
        return;
    }
    buildAdjacencyFlag_ = true;

    // 2) resize properly matrices (if needed)
    r_.resize(obsDim_,1);//dense vector TODO is it better to reserve and push_back??
    A_.resize(obsDim_, stateDim_);//Sparse matrix clears data, but keeps the prev reserved space
    W_.resize(obsDim_, obsDim_);//TODO should we reinitialize this all the time? an incremental should be fairly easy



    // 3) Evaluate every factor given the current state and bookeeping of Factor indices
    std::vector<uint_t> reservationA;
    reservationA.reserve( obsDim_ );
    std::vector<uint_t> reservationW;
    reservationW.reserve( obsDim_ );
    std::vector<factor_id_t> indFactorsMatrix;
    indFactorsMatrix.reserve(factors_.size());
    M_ = 0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        f->evaluate_residuals();
        f->evaluate_jacobians();
        f->evaluate_chi2();

        // calculate dimensions for reservation and bookeping vector
        uint_t dim = f->get_dim_obs();
        uint_t allDim = f->get_all_nodes_dim();
        for (uint_t j = 0; j < dim; ++j)
        {
            reservationA.push_back(allDim);
            reservationW.push_back(dim-j);//XXX this might be allocating more elements than necessary, check
        }
        indFactorsMatrix.push_back(M_);
        M_ += dim;
    }
    assert(M_ == obsDim_ && "FGraphSolve::buildAdjacency: Observation dimensions are not coincident\n");
    A_.reserve(reservationA); //Exact allocation for elements.
    W_.reserve(reservationW); //same


    // XXX This could be subject to parallelization, maybe on two steps: eval + build
    for (factor_id_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];

        // 4) Get the calculated residual
        r_.block(indFactorsMatrix[i], 0, f->get_dim_obs(), 1) <<  f->get_residual();

        // 5) build Adjacency matrix as a composition of rows
        // 5.1) Get the number of nodes involved. It is a vector of nodes
        auto neighNodes = f->get_neighbour_nodes();
        // Iterates over the Jacobian row
        for (uint_t l=0; l < f->get_dim_obs() ; ++l)
        {
            uint_t totalK = 0;
            // Iterates over the number of neighbour Nodes (ordered by construction)
            for (uint_t j=0; j < neighNodes->size(); ++j)
            {
                uint_t dimNode = (*neighNodes)[j]->get_dim();
                // check for node if it is an anchor node, then skip emplacement of Jacobian in the Adjacency
                if ((*neighNodes)[j]->get_node_mode() == Node::nodeMode::ANCHOR)
                {
                    totalK += dimNode;// we need to account for the dim in the Jacobian, to read the next block
                    continue;//skip this loop
                }
                factor_id_t id = (*neighNodes)[j]->get_id();
                for(uint_t k = 0; k < dimNode; ++k)
                {
                    // order according to the permutation vector
                    uint_t iRow = indFactorsMatrix[i] + l;
                    // In release mode, indexes outside will not trigger an exception
                    uint_t iCol = indNodesMatrix_[id] + k;
                    // This is an ordered insertion
                    A_.insert(iRow,iCol) = f->get_jacobian()(l, k + totalK);
                }
                totalK += dimNode;
            }
        }


        // 5) Get information matrix for every factor
        // For robust factors, here is where the robust weights should be applied
        matData_t robust_weight = 1.0;
        for (uint_t l = 0; l < f->get_dim_obs(); ++l)
        {
            // only iterates over the upper triangular part
            for (uint_t k = l; k < f->get_dim_obs(); ++k)
            {
                uint_t iRow = indFactorsMatrix[i] + l;
                uint_t iCol = indFactorsMatrix[i] + k;
                // Weights are then applied both to the residual and the Hessian by modifying the information matrix.
                robust_weight = f->evaluate_robust_weight(std::sqrt(f->get_chi2()));
                W_.insert(iRow,iCol) = robust_weight * f->get_information_matrix()(l,k);
            }
        }
    } //end factors loop


}

void FGraphSolve::build_info_adjacency()
{
    /**
     * L_ dx = b_ corresponds to the normal equation A'*W*A dx = A'*W*r
     * only store the lower part of the information matrix (symmetric)
     *
     * XXX: In terms of speed, using the selfadjointview does not improve,
     * Eigen stores a temporary object and then copy only the upper part.
     *
     */
    // check for a problem built
    if (buildAdjacencyFlag_)
    {
        L_ = (A_.transpose() * W_.selfadjointView<Eigen::Upper>() * A_);
        b_ = A_.transpose() * W_.selfadjointView<Eigen::Upper>() * r_;
    }

    // If any EF, we should combine both solutions
    if (eigen_factors_.size() > 0 )
    {
        if (buildAdjacencyFlag_)
        {
            L_ += hessianEF_.selfadjointView<Eigen::Upper>();
            b_ += gradientEF_;
        }
        // case when there are pure EF and no other factor
        else
        {
            L_ = hessianEF_.selfadjointView<Eigen::Upper>();
            b_ = gradientEF_;
        }
    }
}


void FGraphSolve::build_info_EF()
{
    gradientEF_.resize(stateDim_,1);
    gradientEF_.setZero();
    std::vector<Triplet> hessianData;
    // TODO if EF ever connected a node that is not 6D, then this will not hold.
    hessianData.reserve(eigen_factors_.size()*21);//For each EF we reserve the uppder triangular view => 6+5+..+1  = 21

    for (size_t id = 0; id < eigen_factors_.size(); ++id)
    {
        auto f = eigen_factors_[id];
        f->evaluate_residuals();
        f->evaluate_jacobians();//and Hessian
        f->evaluate_chi2();
        auto neighNodes = f->get_neighbour_nodes();
        for (auto node : *neighNodes)
        {
            uint_t indNode = node->get_id();
            if (node->get_node_mode() == Node::nodeMode::ANCHOR)
            {
                continue;
            }
            // Updating Jacobian, b should has been previously calculated
            Mat61 J = f->get_jacobian(indNode);

            // Calculate the robust factor contribution, similar than in the adjacency
            // Now the weight fator should be introduced in the Hessian block and in the gradient s.t.
            //        (wH)^-1* wgrad
            // When using the aggregated matrix of all EFs, this operation is not trivila (for one EF is trival, it cancels out ofc)
            matData_t robust_weight = 1.0;
            // The vlaue needs to be normalized by
            robust_weight = f->evaluate_robust_weight();
            J *= robust_weight;

            // It requires previous calculation of indNodesMatrix (in build adjacency)
            gradientEF_.block<6,1>(indNodesMatrix_[indNode],0) += J;

            // get the neighboiring nodes TODO and for over them
            uint_t startingIndex = indNodesMatrix_[indNode];
            for (auto node2 : *neighNodes)
            {
                // getting second index, rows in the hessian matrix
                uint_t indNode_2 = node2->get_id();
                uint_t startingIndex_2 = indNodesMatrix_[indNode_2];

                // Check if the indexes (col,row) is in the upper triangular part or skip if not
                if (startingIndex_2 < startingIndex)
                    continue;

                // Calculate hessian, this is a lookup
                Mat6 H;
                // If there is no such crosterms, the methods returns false and the block embeding into H is skipped
                if (!f->get_hessian(H,indNode,indNode_2))
                {
                    continue;
                }
                //std::cout << "Hessian node (i,j) = (" << indNode << ", " << indNode_2 << ")\n";

                // Robust factors, for the same EF, we must account for the weight factor in the block hessian as well:
                H *= robust_weight;
                

                // Calculate the variable that allows to control diagonal/crossterms in the for() below
                // If it is a crossterm, it needs all elements of the 6x6 matrix, so it does not enable the for start in diag (=0)
                // If it is diagonal, it can start at the current row for the upper triangular view (therefore =1)
                uint_t cross_term_reset = 0;
                if (indNode == indNode_2)
                    cross_term_reset = 1;


                // Updating the Full Hessian
                // XXX if EF ever connected a node that is not 6D, then this will not hold. TODO
                for (uint_t i = 0; i < 6; i++)
                {
                    // for diagonal terms, this will start at j=i, which give the uppter triang view
                    for (uint_t j = i*cross_term_reset; j<6; j++)
                    {
                        // convert the hessian to triplets, duplicated ones will be summed
                        // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#a8f09e3597f37aa8861599260af6a53e0
                        hessianData.emplace_back(Triplet(startingIndex+ i, startingIndex_2+ j, H(i,j)));
                    }
                }

            }


        }
    }

    // create a Upper-view sparse matrix from the triplets:
    hessianEF_.resize(stateDim_,stateDim_);
    hessianEF_.setFromTriplets(hessianData.begin(), hessianData.end());
}

matData_t FGraphSolve::chi2(bool evaluateResidualsFlag)
{
    matData_t totalChi2 = 0.0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        if (evaluateResidualsFlag)
        {
            f->evaluate_residuals();
            f->evaluate_chi2();
        }
        totalChi2 += f->get_chi2();
    }

    for (auto &ef : eigen_factors_)
    {
        if (evaluateResidualsFlag)
        {
            ef->evaluate_residuals();
            ef->evaluate_chi2();
        }
        totalChi2 += ef->get_chi2();
    }
    return totalChi2;
}

void FGraphSolve::update_nodes()
{
    int acc_start = 0;
    for (uint_t i = 0; i < active_nodes_.size(); i++)
    {
        // node update is the negative of dx just calculated.
        // x = x - alpha * H^(-1) * Grad = x - dx
        // Depending on the optimization, it is already taking care of the step alpha, so we assume alpha = 1
        auto node_update = -dx_.block(acc_start, 0, active_nodes_[i]->get_dim(), 1);
        active_nodes_[i]->update(node_update);

        acc_start += active_nodes_[i]->get_dim();
    }
}

void FGraphSolve::synchronize_nodes_auxiliary_state()
{
    for (auto &&n : active_nodes_)
        n->set_auxiliary_state(n->get_state());
}


void FGraphSolve::synchronize_nodes_state()
{
    for (auto &&n : active_nodes_)
        n->set_state(n->get_auxiliary_state());
}

// method to output (to python) or other programs the current state of the system.
std::vector<MatX> FGraphSolve::get_estimated_state()
{
    vector<MatX> results;
    results.reserve(nodes_.size());

    for (uint_t i = 0; i < nodes_.size(); i++)
    {
        //nodes_[i]->print();
        MatX updated_pos = nodes_[i]->get_state();
        results.emplace_back(updated_pos);
    }

    return results;
}

MatX1 FGraphSolve::get_chi2_array()
{
    MatX1 results(factors_.size());

    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        results(i) = f->get_chi2();
    }

    return results;
}


std::vector<bool> FGraphSolve::get_eigen_factors_robust_mask()
{
    // it will create a copy on the python side, that is why scope here is just defined in here
    // XXX:one day we should improve the interface with python...
    std::vector<bool> results;
    results.reserve(eigen_factors_.size());

    bool mask;
    for (auto ef : eigen_factors_)
    {
        mask = ef->get_robust_mask();
        results.push_back(mask);
    }

    return results;
}

std::vector<bool> FGraphSolve::get_factors_robust_mask()
{
    std::vector<bool> results;
    results.reserve(factors_.size());

    bool mask;
    for (auto f : factors_)
    {
        mask = f->get_robust_mask();
        results.push_back(mask);
    }

    return results;
}
