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
 * FGraphDiffPy.cpp
 *
 *  Created on: May 28, 2024
 *      Author: Aleksei Panchenko
 *              aleksei.panchenko@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "mrob/factor_graph_diff_solve.hpp"
#include "mrob/factors/nodePose2d.hpp"
#include "mrob/factors/factor1Pose2d_diff.hpp"
#include "mrob/factors/factor2Poses2d_diff.hpp"

//#include <Eigen/Geometry>

namespace py = pybind11;
using namespace mrob;



/**
 * Create auxiliary class to include all functions:
 *    - creates specific factors and nodes (while the cpp maintains a polymorphic data structure)
 *
 */

class FGraphDiffPy : public FGraphDiffSolve
{
public:
    /**
     * Constructor for the python binding. By default uses the Cholesky adjoint solving type.
     * For the robust factor type, it indicates one (default - Quadratic)
     * and all factors will automatically be this kind of robust factors
     *
     * NOTE: in cpp each new factor needs to specify the robust type, which give more freedom, but
     * since the .py is a more "structured" class, it is included in the constructor and that's
     * all the user will ever see
     *
     * TODO: change type of robust? maybe some apps would need that feature...
     */
    FGraphDiffPy(mrob::DiffFactor::robustFactorType robust_type = mrob::DiffFactor::robustFactorType::QUADRATIC) :
        FGraphDiffSolve(FGraphDiffSolve::matrixMethod::ADJ), robust_type_(robust_type) {}
    factor_id_t add_node_pose_2d(const py::EigenDRef<const Mat31> x, mrob::Node::nodeMode mode)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose2d(x,mode));
        this->add_node(n);
        return n->get_id();
    }
    void add_factor_1pose_2d_diff(const py::EigenDRef<const Mat31> obs, uint_t nodeId, const py::EigenDRef<const Mat3> obsInvCov)
    {
        auto n1 = this->get_node(nodeId);
        std::shared_ptr<mrob::DiffFactor> f(new mrob::Factor1Pose2d_diff(obs,n1,obsInvCov,robust_type_));
        this->add_factor(f);
    }
    void add_factor_2poses_2d_diff(const py::EigenDRef<const Mat31> obs, uint_t nodeOriginId, uint_t nodeTargetId,
            const py::EigenDRef<const Mat3> obsInvCov, bool updateNodeTarget)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::DiffFactor> f(new mrob::Factor2Poses2d_diff(obs,nO,nT,obsInvCov, updateNodeTarget,robust_type_));
        this->add_factor(f);
    }
    void add_factor_2poses_2d_odom_diff(const py::EigenDRef<const Mat31> obs, uint_t nodeOriginId, uint_t nodeTargetId, const py::EigenDRef<const Mat3> obsInvCov)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::DiffFactor> f(new mrob::Factor2Poses2dOdom_diff(obs,nO,nT,obsInvCov,true,robust_type_));//true is to update the node value according to obs
        this->add_factor(f);
    }

private:
    mrob::DiffFactor::robustFactorType robust_type_;
};

void init_FGraphDiff(py::module &m)
{
    py::enum_<FGraphDiffSolve::optimMethod>(m, "FGraphDiff.optimMethod")
        .value("FGraphDiff_GN", FGraphDiffSolve::optimMethod::GN)
        .value("FGraphDiff_LM", FGraphDiffSolve::optimMethod::LM)
        .value("FGraphDiff_LM_ELLIPS", FGraphDiffSolve::optimMethod::LM_ELLIPS)
        .export_values()
        ;
//     py::enum_<DiffFactor::robustFactorType>(m, "FGraphDiff.robustFactorType")
//         .value("QUADRATIC", DiffFactor::robustFactorType::QUADRATIC)
//         .value("CAUCHY", DiffFactor::robustFactorType::CAUCHY)
//         .value("HUBER", DiffFactor::robustFactorType::HUBER)
//         .value("MCCLURE", DiffFactor::robustFactorType::MCCLURE)
//         .value("RANSAC", DiffFactor::robustFactorType::RANSAC)
//         .export_values()
//         ;
//     py::enum_<Node::nodeMode>(m, "FGraphDiff.nodeMode")
//     .value("NODE_STANDARD", Node::nodeMode::STANDARD)
//     .value("NODE_ANCHOR", Node::nodeMode::ANCHOR)
//     .value("NODE_SCHUR_MARGI", Node::nodeMode::SCHUR_MARGI)
//     .export_values()
//     ;
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<FGraphDiffPy> (m,"FGraphDiff")
            .def(py::init<DiffFactor::robustFactorType>(),
                    "Constructor, solveType default is ADJ and robust factor is quadratic.",
                    py::arg("robust_type") =  DiffFactor::robustFactorType::QUADRATIC)
            .def("solve", &FGraphDiffSolve::solve,
                    "Solves the corresponding FG.\n"
                    "Options:\n method = mrob.GN (Gauss Newton). It carries out a SINGLE iteration.\n"
                    "                  = mrob.LM (Levenberg-Marquard), default option,it has several parameters:\n"
                    " - marIters = 20 (by default). Only for LM\n"
                    " - lambda = 1-5, LM paramter for the size of the update\n"
                    " - solutionTolerance: convergence criteria\n"
                    " - verbose: by default false. If you want output on optim, set to true.",
                    py::arg("method") =  FGraphDiffSolve::optimMethod::LM,
                    py::arg("maxIters") = 20,
                    py::arg("lambdaParam") = 1e-5,
                    py::arg("solutionTolerance") = 1e-6,
                    py::arg("verbose") = false)
            .def("chi2", &FGraphDiffSolve::chi2,
                    "Calculated the chi2 of the problem.\n"
                    "By default re-evaluates residuals, \n"
                    "if set to false if doesn't:    evaluateResidualsFlag = False",
                    py::arg("evaluateResidualsFlag") = true)
            .def("get_estimated_state", &FGraphDiffSolve::get_estimated_state,
                    "returns the list of states ordered according to ids.\n"
                    "Each state can be of different size and some of these elements might be matrices if the are 3D poses")
            .def("get_information_matrix", &FGraphDiffSolve::get_information_matrix,
                    "Returns the information matrix (sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_adjacency_matrix", &FGraphDiffSolve::get_adjacency_matrix,
                    "Returns the adjacency matrix (sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_W_matrix", &FGraphDiffSolve::get_W_matrix,
                    "Returns the W matrix of observation noises(sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_vector_b", &FGraphDiffSolve::get_vector_b,
                    "Returns the vector  b = A'Wr, from residuals. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_chi2_array", &FGraphDiffSolve::get_chi2_array,
                    "Returns the vector of chi2 values for each factor. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_factors_robust_mask", &FGraphDiffSolve::get_factors_robust_mask,
                    "Returns a vector (python list) of factors robust functions: - True if the robust mask was applied - False if the robust factor had not effect",
                    py::return_value_policy::copy)
            .def("number_nodes", &FGraphDiffSolve::number_nodes, "Returns the number of nodes")
            .def("number_factors", &FGraphDiffSolve::number_factors, "Returns the number of factors")
            .def("print", &FGraphDiff::print, "By default False: does not print all the information on the Fgraph", py::arg("completePrint") = false)
            .def("get_dchi2_dz", &FGraphDiffSolve::get_dchi2_dz,
                   "Calculate chi2 gradient with reference to all obzervations z in all factors")
            // Robust factors GUI
            // TODO, we want to set a default robust function? maybe at ini?
            // TODO we want a way to change the robust factor for each node, maybe accesing by id? This could be away to inactivate factors...
            // -----------------------------------------------------------------------------
            // Specific call to 2D
            .def("add_node_pose_2d", &FGraphDiffPy::add_node_pose_2d,
                    " - arguments, initial estimate (np.zeros(3)\n"
                    "output, node id, for later usage",
                    py::arg("x"),
                    py::arg("mode") = Node::nodeMode::STANDARD)
            .def("add_factor_1pose_2d_diff", &FGraphDiffPy::add_factor_1pose_2d_diff)
            .def("add_factor_2poses_2d_diff", &FGraphDiffPy::add_factor_2poses_2d_diff,
                    "Factors connecting 2 poses. If last input set to true (by default false), also updates "
                    "the value of the target Node according to the new obs + origin node",
                    py::arg("obs"),
                    py::arg("nodeOriginId"),
                    py::arg("nodeTargetId"),
                    py::arg("obsInvCov"),
                    py::arg("updateNodeTarget") = false)
            .def("add_factor_2poses_2d_odom_diff", &FGraphDiffPy::add_factor_2poses_2d_odom_diff,
                    "add_factor_2poses_2d_odom(obs, nodeOriginId, nodeTargetId, W)"
                    "\nFactor connecting 2 poses, following an odometry model."
                    "\nArguments are obs, nodeOriginId, nodeTargetId and obsInvCov",
                    py::arg("obs"),
                    py::arg("nodeOriginId"),
                    py::arg("nodeTargetId"),
                    py::arg("obsInvCov"))
            ;

}
