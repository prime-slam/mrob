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
#include "mrob/factor_graph.hpp"

using namespace mrob;

int main() {
    std::cout << "=== FactorLidarMap2PoseInterpolation Test Program ===" << std::endl;
    
    try {
        
        std::cout << "\n1. Creating test data..." << std::endl;
        
        
        Mat51 observation;
        observation << 1.5, 2.0, 0.8, 0.0, 0.5; // [x, y, z, 0, time]
        
        
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
        
        FGraph graph;
        std::shared_ptr<Node> nodeOrigin = std::make_shared<NodePose3dVelTc>(state_origin);
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
        
        auto factor = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, obsInf, Factor::robustFactorType::QUADRATIC
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
        factor->print();
        
        
        std::cout << "\n8. Testing interpolation..." << std::endl;
        
        SE3tc T_origin(state_origin);
        SE3tc T_target(state_target);
        matData_t t_obs = observation(4);
        
        std::cout << "Origin time: " << T_origin.time() << std::endl;
        std::cout << "Target time: " << T_target.time() << std::endl;
        std::cout << "Observation time: " << t_obs << std::endl;
        
        
        std::cout << "\n9. Testing with different observation times..." << std::endl;
        
        for (double t = 0.0; t <= 1.0; t += 0.2) {
            Mat51 obs_t = observation;
            obs_t(4) = t;  // Set observation time
            
            auto factor_t = std::make_shared<FactorLidarMap2PoseInterpolation>(
                obs_t, map_point, nodeOrigin, nodeTarget, obsInf, Factor::robustFactorType::QUADRATIC
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
