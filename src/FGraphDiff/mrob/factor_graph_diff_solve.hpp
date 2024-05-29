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
 * factor_graph_diff_solve.hpp
 *
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SRC_FACTOR_GRAPH_DIFF_SOLVE_HPP_
#define SRC_FACTOR_GRAPH_DIFF_SOLVE_HPP_


#include "mrob/factor_graph_diff.hpp"
#include "mrob/time_profiling.hpp"
#include <unordered_map>

namespace mrob {


/**
 * Class FGraphDiffSolve creates all the required matrices for solving the LSQ problem.
 * The problem takes the following form:
 *
 * x* = argmin {C(x)} = argmin {1/2 sum ||r_i(x,z)||2_W} = argmin 1/2||r||2_W.
 *
 * last term in vectorized form.
 *
 * By convention, the residuals r_i are ALWAYS formulated as follows:
 * -------------------------------------------
 * |           r(x) =  h(x) - z              |
 * -------------------------------------------
 *
 * With this arrangement, the linearized factor substracts the residual (r)
 * to the first order term of the nonlinear observation function:
 * ||h(x)-z||2_W = ||h(x0) + J dx - z ||2_W = ||J dx + r||2_W
 *
 * When optimizing the linearized LSQ:
 * dC    1  d
 * --  = - ---(sum r' W r) = J' W (J dx + r) = 0
 * dx    2  dx
 *
 *    => dx = -(J'WJ)^(-1) J'W r
 *
 * This convention will be followed by all factors in this library, otherwise the optimization
 * will not work properly.
 *
 * Different options are provided:
 * 	- ADJ: Adjacency matrix (plus indirect construction of Information)
 * 	- SCHUR (TODO): Diagonal and information from Schur complement
 *
 * Routines provide different optimization methods:
 *  - Gauss-Newton (GN) using Cholesky LDLT with minimum degree ordering
 *  - Levenberg–Marquardt (LM) (Nocedal Ch.10) using spherical
 *                     trust region alg. (Nocedal 4.1) to estimate a "good" lambda.
 *                     Bertsekas p.105 proposes a similar heuristic approach for the trust
 *                     region, which we convert to lambda estimation (we follow Bertsekas' notation in code).
 *  - LM_Ellipsoid implementation. Slightly different than LM-Spherical on how to condition the information matrix.
 */
class FGraphDiffSolve: public FGraphDiff
{
public:
    /**
     * This enums all matrix building methods available
     * For now we only do build adjacency
     */
    enum matrixMethod{ADJ=0, SCHUR};
    /**
     * This enums optimization methods available:
     *  - Gauss Newton
     *  - Levenberg Marquardt (trust-region-like for lambda adjustment)
     *  - LM elliptical J'WJ + lambda * diag(J'WJ)
     */
    enum optimMethod{GN=0, LM, LM_ELLIPS};

    FGraphDiffSolve(matrixMethod method = ADJ);
    virtual ~FGraphDiffSolve();

    /**
     * Solve call the corresponding routine on the class parameters or
     * ultimately on the function input,
     * by default optim method is Levenberg-Marquardt
     *
     * Return: number of iterations
     *         Failed to converge = 0 iterations
     */
    uint_t solve(optimMethod method = LM, uint_t maxIters = 10,
                matData_t lambda = 1e-6, matData_t solutionTolerance = 1e-2,
                bool verbose = false);
    /**
     * Evaluates the current solution chi2.
     *
     * Variable relinearizeProblemFlag:
     *      - (default) true: Recalculates residuals.
     *      - false: Uses the previous calculated residuals
     */
    matData_t chi2(bool evaluateResidualsFlag = true);
    /**
     * Returns a Reference to the solution vector
     * of all variables, vectors, matrices, etc.
     */
    std::vector<MatX> get_estimated_state();

    /**
     * Functions to set the matrix method building
     */
    void set_build_matrix_method(matrixMethod method) {matrixMethod_ = method;};
    matrixMethod get_build_matrix_method() { return matrixMethod_;};

    /**
     * Returns a copy to the information matrix.
     * pybind does not allow to pass by reference, so there is a copy anyway
     * CHeck out more here: https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html
     * TODO If true, it re-evaluates the problem
     */
    SMatCol get_information_matrix() { return L_;}
    /**
     * Returns a copy to the Adjacency matrix.
     * There is a conversion (implies copy) from Row to Col-convention (which is what np.array needs)
     * TODO If true, it re-evaluates the problem
     */
    SMatCol get_adjacency_matrix() { return A_;}
    /**
     * Returns a copy to the W matrix.
     * There is a conversion (implies copy) from Row to Col-convention (which is what np.array needs)
     * TODO If true, it re-evaluates the problem
     */
    SMatCol get_W_matrix() { return W_;}
    /**
     * Returns a copy to the processed residuals in state space b = A'Wr.
     * TODO If true, it re-evaluates the problem
     */
    MatX1 get_vector_b() { return b_;}
    /**
     * Returns a vector of chi2 values for each of the factors.
     */
    MatX1 get_chi2_array();
    /**
     * Returns a vector (python list)
     * - True if the robust mask was applied
     * - False if the robust factor had not effect.
     *
     * The index in the graph is the factor Id
    */
    std::vector<bool> get_factors_robust_mask();

protected:
    /**
     * build problem creates an information matrix L, W and a vector b
     *
     * It chooses from building the information from the adjacency matrix,
     * directly building info or schur (TODO)
     *
     * If bool useLambda is true, it also stores a vector D2 containing the diagonal
     * of the information matrix L
     */
    void build_problem(bool useLambda = false);
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and creates a block diagonal matrix W with each factors information.
     * As a result, residuals, Jacobians and chi2 values are up to date
     *
     */
    void build_adjacency();
    /**
     * From the adjacency matrix it creates the information matrix as
     *              L = A^T * W * A
     * The residuals are also calculated as b = A^T * W *r
     */
    void build_info_adjacency();
    void build_schur(); // TODO

    /**
     * Once the matrix L is generated, it solves the linearized LSQ
     * by using the Gauss-Newton algorithm
     *
     * Input useLambda (default false) builds the GN problem with lambda factor on the diagonal
     *    L = A'*A + lambda * I
     */
    void optimize_gauss_newton(bool useLambda = false);

    /**
     * It generates the information matrix as
     *              L' = L +  lambda * I
     *
     * TODO, the preconditioning could be adapted to expected values, such as w < pi and v < avg
     *               L' = L +  lambda * D2
     *
     * Iteratively updates the solution given the right estimation of lambda
     * Parameters are necessary to be specified in advance, o.w. it would use default values.
     *
     * input maxIters, before returning a result
     *
     * output: number of iterations it took to converge.
     *    0 when incorrect solution
     */
    uint_t optimize_levenberg_marquardt(uint_t maxIters);

    /**
     * Function that updates all nodes with the current solution,
     * this must be called after solving the problem
     */
    void update_nodes();

    /**
     * Synchronize state variable in all nodes
     * exactly value the current state.
     *
     * Usually this function un-does an incorrect update of the state.
     */
    void synchronize_nodes_state();

    /**
     * Synchronize auxiliary state variables in all nodes
     * exactly value the current state.
     * This function is used when we will update a solution
     * but it needs verification, so we book-keep at auxiliary.
     */
    void synchronize_nodes_auxiliary_state();

    /**
     * This function build the node index that will be used for creating the
     * adjacency matrix and the information matrix.
     *
     * It maps the node Id to the column index in the matrix, such that
     * non-consecutive nodes can be bookkeep for a fast access
     */
    void build_index_nodes_matrix();

    // Variables for solving the FGraphDiff
    matrixMethod matrixMethod_;
    optimMethod optimMethod_;


    factor_id_t N_; // total number of state variables
    factor_id_t M_; // total number of observation variables

    std::unordered_map<factor_id_t, factor_id_t > indNodesMatrix_;

    SMatRow A_; //Adjacency matrix, as a Row sparse matrix. The reason is for filling in row-fashion for each factor
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMatCol L_; //Information matrix. For Eigen Cholesky AMD Ordering it is necessary Col convention for compilation.
    MatX1 b_; // Post-processed residuals, A'*W*r

    // Correction deltas
    MatX1 dx_;

    // Particular parameters for Levenberg-Marquard
    matData_t lambda_; // current value of lambda
    matData_t solutionTolerance_;
    MatX1 diagL_; //diagonal matrix (vector) of L to update it efficiently


    // Methods for handling Eigen factors. If not used, no problem
    bool buildAdjacencyFlag_;

    // time profiling
    TimeProfiling time_profiles_;

    // printing flag
    bool verbose_;
};


}


#endif /* SRC_FACTOR_GRAPH_DIFF_SOLVE_HPP_ */
