/* Test program for FactorLidarMap2PoseInterpolation
 *
 * This program tests the FactorLidarMap2PoseInterpolation class functionality
 * by creating test nodes and factors, then evaluating residuals and Jacobians.
 */

#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "mrob/factors/factorLidarMap2PoseInterpolation.hpp"
#include "mrob/factors/nodePose3dVelTc.hpp"
#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
#include "mrob/SE3tc.hpp"
//#include "mrob/factor_graph.hpp"
#include "mrob/factor_graph_solve.hpp"

using namespace mrob;

int main() {
    std::cout << "=== FactorLidarMap2PoseInterpolation Test Program ===" << std::endl;
    
    try {
        
        std::cout << "\n1. Creating test data..." << std::endl;
        
        
        Mat41 observation;
        observation << 1.5, 2.0, 0.8,  0.5; // [x, y, z, time]  XXX changed to 4 coordinates
        
        
        Mat31 map_point;
        map_point << 1.0, 1.5, 0.5;
        
        
        Mat3 obsInf = Mat3::Identity() * 10.0; 
        
        std::cout << "Observation: " << observation.transpose() << std::endl;
        std::cout << "Map point: " << map_point.transpose() << std::endl;
        std::cout << "Information matrix:\n" << obsInf << std::endl;
        
        
        std::cout << "\n2. Creating test nodes..." << std::endl;
        
        
        Mat5 state_origin;
        state_origin << 1.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 0.5, 1.0; 

        Mat5 state_target;
        state_target << 0.866, -0.8, 0.0, 1.0, 0.0,   
                        0.5, 0.866, 0.0, 0.5, 0.0,
                        0.0, 0.0, 1.0, 0.2, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.5, 1.0;
        
        FGraphSolve graph;
        std::shared_ptr<Node> nodeOrigin = std::make_shared<NodePose3dVelTc>(state_origin,Node::ANCHOR);//XXX: this is going to be fixed, so the optimziation needs to know it, this is the way
        std::shared_ptr<Node> nodeTarget = std::make_shared<NodePose3dVelTc>(state_target);

        factor_id_t origin_id = graph.add_node(nodeOrigin);
        factor_id_t target_id = graph.add_node(nodeTarget);

        std::cout << "Origin node ID: " << origin_id << std::endl;
        std::cout << "Target node ID: " << target_id << std::endl;

        Mat5 X_origin;
        X_origin = nodeOrigin->get_state();    
        Mat5 X_target;
        X_target = nodeTarget->get_state();
        std::cout << "Origin node state number: \n" << X_origin << std::endl;
        std::cout << "Target node state:\n" << X_target << std::endl;
        

        std::cout << "\n3. Creating FactorLidarMap2PoseInterpolation..." << std::endl;
        
        // Create offset_lidar_imu transformation (identity for simplicity)
        Mat4 offset_lidar_imu = Mat4::Identity();
        
        // I saw you ave to cast it anyway in FGraph example, so just define polimorphism one time at the declaration of the variable.
        std::shared_ptr<Factor> factor = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC
        );
        
        std::cout << "Factor created successfully!" << std::endl;
        
        
        std::cout << "\n4. Evaluating residuals..." << std::endl;
        
        factor->evaluate_residuals();
        
        Mat31 residual = factor->get_residual();
        
        std::cout << "Residual: " << residual.transpose() << std::endl;
        
        
        std::cout << "\n5. Evaluating Jacobians..." << std::endl;
        
        factor->evaluate_jacobians();
        Mat<3,9> jacobian = factor->get_jacobian(0);
        
        std::cout << "Jacobian:\n" << jacobian << std::endl;

        std::cout << "\n6. Evaluating chi2..." << std::endl;
        
        factor->evaluate_chi2();
        double chi2 = factor->get_chi2();
        
        std::cout << "Chi2 error: " << chi2 << std::endl;
        
        
        std::cout << "\n7. Factor details:" << std::endl;
        factor->print();//all printed above, this function gives
        
        
        std::cout << "\n8. Testing interpolation..." << std::endl;
        
        SE3tc T_origin(state_origin);
        SE3tc T_target(state_target);
        matData_t t_obs = observation(3);
        
        std::cout << "Origin time: " << T_origin.time() << std::endl;
        std::cout << "Target time: " << T_target.time() << std::endl;
        std::cout << "Observation time: " << t_obs << std::endl;//this should be in [target,origin]. it must coincide, so carafull with sweeping this variable.
        
        //to really test it works properly, I think you want to solve this graph
        observation << 1.5, 1.0, 0.0,  1.0;
        map_point << 1.5, 2.0, 0.20;
        std::shared_ptr<Factor> factor_1 = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC);
        graph.add_factor(factor_1);
        
        observation << -1.5, 1.0, 0.0,  1.0;
        map_point << -1.5, 2.0, 0.20;
        std::shared_ptr<Factor> factor_2 = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC);
        graph.add_factor(factor_2);

        observation << 1.5, 2.0, 0.0,  1.0;
        map_point << 1.5, 3.0, 0.20;
        std::shared_ptr<Factor> factor_3 = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC);
        graph.add_factor(factor_3);

        // solve function in fg_solve.cpp. There are defauls params, but not using them
        graph.print(true);
        auto iters = 
                graph.solve(FGraphSolve::optimMethod::LM,
                    20,
                    1e-5,
                    1e-6,
                    true);

        std::cout << "\nSolved, chi2 = " << graph.chi2() << ", iters = " << iters <<std::endl;
    
        graph.print(true);

        
        std::cout << "\n9. Testing with different observation times..." << std::endl;
        
        for (double t = 0.0; t <= 1.0; t += 0.2) {
            Mat41 obs_t = observation;
            obs_t(3) = t;  // Set observation time
            
            auto factor_t = std::make_shared<FactorLidarMap2PoseInterpolation>(
                obs_t, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC
            );
            
            factor_t->evaluate_residuals();
            Mat31 res_t = factor_t->get_residual();
            
            std::cout << "Time " << t << ": residual = " << res_t.transpose() << std::endl;
        }
        
        std::cout << "\n=== Test completed successfully! ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred!" << std::endl;
        return 1;
    }
    
    return 0;
}
