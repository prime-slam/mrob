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
 * FGraphTimePy.cpp
 *
 *  Created on: Sep 11, 2024
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/nodePose3dVelTc.hpp"
#include "mrob/factors/nodeLandmark3d.hpp"
#include "mrob/factors/factor1Pose3dVel.hpp"
#include "mrob/factors/factor2Poses3dTwistMatching.hpp"
#include "mrob/factors/factorCameraProj3dPoint.hpp"



namespace py = pybind11;
using namespace mrob;

/**
 * Create auxiliary class to include all functions:
 *    - creates specific factors and nodes (while the cpp maintains a polymorphic data structure)
 *
 */

class FGraphTimePy : public FGraphSolve
{
public:
    /**
     * Constructor for the python binding. See Fgraph.
     * 
     * New, adding methods to consider time continious solutions
     */
    FGraphTimePy(mrob::Factor::robustFactorType robust_type = mrob::Factor::robustFactorType::QUADRATIC) :
        FGraphSolve(FGraphSolve::matrixMethod::ADJ), robust_type_(robust_type) {}
    factor_id_t add_node_pose_3d_vel_tc(const SE3tc &x, double time_stamp, mrob::Node::nodeMode mode)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose3dVelTc(x,mode));
        n->set_time_stamp(time_stamp);
        this->add_node(n);
        return n->get_id();
    }
    factor_id_t add_node_landmark_3d(const py::EigenDRef<const Mat31> x, mrob::Node::nodeMode mode)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodeLandmark3d(x,mode));
        this->add_node(n);
        return n->get_id();
    }
    factor_id_t add_factor_1pose_3d(const SE3 &obs, uint_t nodeId, double time_stamp, const py::EigenDRef<const Mat6> obsInvCov)
    {
        auto n1 = this->get_node(nodeId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose3dVel(obs,n1,obsInvCov,robust_type_));
        this->add_factor(f);
        f->set_time_stamp(time_stamp);
        return f->get_id();
    }
    factor_id_t add_factor_2pose_3d_twist(const py::EigenDRef<const Mat61> obs, uint_t nodeOrigin, uint_t nodeTarget,  double time_stamp, const py::EigenDRef<const Mat9> obsInvCov)
    {
        auto n1 = this->get_node(nodeOrigin);
        auto n2 = this->get_node(nodeTarget);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses3dTwistMatching(obs,n1,n2,obsInvCov,robust_type_));
        f->set_time_stamp(time_stamp);//not really used since the twist is constant in the interval.
        this->add_factor(f);
        return f->get_id();
    }
    // Visual factors
    // --------------------------------------------------
    // add_factor_camera_proj_3d_point
    /*factor_id_t add_factor_camera_proj_3d_point(const py::EigenDRef<const Mat21> obs, uint_t nodePoseId,
                uint_t nodeLandmarkId, const py::EigenDRef<const Mat41> camera_k, double time_stamp,
                const py::EigenDRef<const Mat2> obsInvCov)
    {
        auto n1 = this->get_node(nodePoseId);
        auto n2 = this->get_node(nodeLandmarkId);
        std::shared_ptr<mrob::Factor> f(new mrob::FactorCameraProj3dPoint(obs,n1,n2,camera_k,obsInvCov,robust_type_));
        this->add_factor(f);
        f->set_time_stamp(time_stamp);
        return f->get_id();
    }*/
private:
    mrob::Factor::robustFactorType robust_type_;
};



void init_FGraphTime(py::module &m)
{
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<FGraphTimePy> (m,"FGraphTime")
            .def(py::init<Factor::robustFactorType>(),
                    "Constructor, solveType default is ADJ and robust factor is quadratic.",
                    py::arg("robust_type") =  Factor::robustFactorType::QUADRATIC)
            .def("solve", &FGraphSolve::solve,
                    "Solves the corresponding FG.\n"
                    "Options:\n method = mrob.GN (Gauss Newton). It carries out a SINGLE iteration.\n"
                    "                  = mrob.LM (Levenberg-Marquard), default option,it has several parameters:\n"
                    " - marIters = 20 (by default). Only for LM\n"
                    " - lambda = 1-5, LM paramter for the size of the update\n"
                    " - solutionTolerance: convergence criteria\n"
                    " - verbose: by default false. If you want output on optim, set to true.",
                    py::arg("method") =  FGraphSolve::optimMethod::LM,
                    py::arg("maxIters") = 20,
                    py::arg("lambdaParam") = 1e-5,
                    py::arg("solutionTolerance") = 1e-6,
                    py::arg("verbose") = false)
            .def("chi2", &FGraphSolve::chi2,
                    "Calculated the chi2 of the problem.\n"
                    "By default re-evaluates residuals, \n"
                    "if set to false if doesn't:    evaluateResidualsFlag = False",
                    py::arg("evaluateResidualsFlag") = true)
            .def("get_estimated_state", &FGraphSolve::get_estimated_state,
                    "returns the list of states ordered according to ids.\n"
                    "Each state can be of different size and some of these elements might be matrices if the are 3D poses")
            .def("get_information_matrix", &FGraphSolve::get_information_matrix,
                    "Returns the information matrix (sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_adjacency_matrix", &FGraphSolve::get_adjacency_matrix,
                    "Returns the adjacency matrix (sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_W_matrix", &FGraphSolve::get_W_matrix,
                    "Returns the W matrix of observation noises(sparse matrix). It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_vector_b", &FGraphSolve::get_vector_b,
                    "Returns the vector  b = A'Wr, from residuals. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_chi2_array", &FGraphSolve::get_chi2_array,
                    "Returns the vector of chi2 values for each factor. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_eigen_factors_robust_mask", &FGraphSolve::get_eigen_factors_robust_mask,
                    "Returns a vector (python list) of Eigen factors robust functions: - True if the robust mask was applied - False if the robust factor had not effect",
                    py::return_value_policy::copy)
            .def("get_factors_robust_mask", &FGraphSolve::get_factors_robust_mask,
                    "Returns a vector (python list) of factors robust functions: - True if the robust mask was applied - False if the robust factor had not effect",
                    py::return_value_policy::copy)
            .def("number_nodes", &FGraphSolve::number_nodes, "Returns the number of nodes")
            .def("number_factors", &FGraphSolve::number_factors, "Returns the number of factors")
            .def("print", &FGraph::print, "By default False: does not print all the information on the Fgraph", py::arg("completePrint") = false)

            // -----------------------------------------------------------------------------
            // Specific call to 3D
            .def("add_node_pose_3d", &FGraphTimePy::add_node_pose_3d_vel_tc,
                    "Input are a pose in 3D, and later it is lifted to SE3vel to match the node.",
                    py::arg("x"),
                    py::arg("time_stamp"),
                    py::arg("mode") = Node::nodeMode::STANDARD)
            .def("add_node_landmark_3d", &FGraphTimePy::add_node_landmark_3d,
                    "Ladmarks are 3D points, in [x,y,z]",
                    py::arg("x"),
                    py::arg("mode") = Node::nodeMode::STANDARD)
            .def("add_factor_1pose_3d", &FGraphTimePy::add_factor_1pose_3d,
                    "Adds a factor observing one pose, a GPS-like factor",
                    py::arg("obs"),
                    py::arg("nodeId"),
                    py::arg("time_stamp"),
                    py::arg("obsInvCov"))
            .def("add_factor_2pose_3d_twist", &FGraphTimePy::add_factor_2pose_3d_twist,
                    "\n Factor for the IMU matching in the interval by two poses",
                    py::arg("obs"),
                    py::arg("nodeOrigin"),
                    py::arg("nodeTarget"),
                    py::arg("time_stamp"),
                    py::arg("obsInvCov"))
            ;
}