#include <iostream>
#include <memory>
#include <Eigen/Dense>

// Include the necessary mrob headers
#include "mrob/factors/factorLidarMap2PoseInterpolation.hpp"
#include "mrob/factors/nodePose3dVelTc.hpp"
#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
#include "mrob/SE3tc.hpp"
#include "mrob/factor_graph.hpp"

using namespace mrob;

int main() {
    std::cout << "=== Factor Graph Test for FactorLidarMap2PoseInterpolation ===" << std::endl;
    
    try {
        // Test 1: Create test data
        std::cout << "\n1. Creating test data..." << std::endl;
        
        // Create observation: [X, Y, Z, 0, t] - lidar observation at time t
        Mat51 observation;
        observation << 1.5, 2.0, 0.8, 0.0, 0.5; // [x, y, z, 0, time]
        
        // Create map point: [X, Y, Z] - 3D point in map
        Mat31 map_point;
        map_point << 1.0, 1.5, 0.5;
        
        // Create information matrix (inverse of covariance)
        Mat3 obsInf = Mat3::Identity() * 10.0; // High precision
        
        std::cout << "Observation: " << observation.transpose() << std::endl;
        std::cout << "Map point: " << map_point.transpose() << std::endl;
        std::cout << "Information matrix:\n" << obsInf << std::endl;
        
        // Test 2: Create factor graph
        std::cout << "\n2. Creating factor graph..." << std::endl;
        
        auto graph = std::make_shared<FGraph>();
        
        // Test 3: Create and add nodes to graph
        std::cout << "\n3. Creating and adding nodes to graph..." << std::endl;
        
        // Create initial poses for origin and target nodes
        Mat5 origin_mat = Mat5::Identity();
        Mat5 target_mat = Mat5::Identity();
        target_mat(0,0) = 0.866; target_mat(0,1) = -0.5; target_mat(0,3) = 1.0;
        target_mat(1,0) = 0.5; target_mat(1,1) = 0.866; target_mat(1,3) = 0.5;
        target_mat(2,3) = 0.2;
        
        SE3tc T_origin_init(origin_mat, 0.0);
        SE3tc T_target_init(target_mat, 1.0);
        
        Mat5 state_origin = T_origin_init.T();
        Mat5 state_target = T_target_init.T();
        
        // Create node pointers
        std::shared_ptr<Node> nodeOrigin = std::make_shared<NodePose3dVelTc>(state_origin);
        std::shared_ptr<Node> nodeTarget = std::make_shared<NodePose3dVelTc>(state_target);
        
        // Add nodes to graph
        graph->add_node(nodeOrigin);
        graph->add_node(nodeTarget);
        
        std::cout << "Origin node ID: " << nodeOrigin->get_id() << std::endl;
        std::cout << "Target node ID: " << nodeTarget->get_id() << std::endl;
        
        std::cout << "Origin node state:\n" << nodeOrigin->get_state() << std::endl;
        std::cout << "Target node state:\n" << nodeTarget->get_state() << std::endl;
        
        // Test 4: Create and add factor to graph
        std::cout << "\n4. Creating and adding factor to graph..." << std::endl;
        
        // Create offset_lidar_imu transformation (identity for simplicity)
        Mat4 offset_lidar_imu = Mat4::Identity();
        
        auto factor = std::make_shared<FactorLidarMap2PoseInterpolation>(
            observation, map_point, nodeOrigin, nodeTarget, offset_lidar_imu, obsInf, Factor::robustFactorType::QUADRATIC
        );
        
        // Add factor to graph
        std::shared_ptr<Factor> factor_ptr = factor;
        graph->add_factor(factor_ptr);
        
        std::cout << "Factor created and added to graph!" << std::endl;
        std::cout << "Factor ID: " << factor->get_id() << std::endl;
        
        // Test 5: Print graph information
        std::cout << "\n5. Graph information:" << std::endl;
        std::cout << "Number of nodes: " << graph->number_nodes() << std::endl;
        std::cout << "Number of factors: " << graph->number_factors() << std::endl;
        
        // Test 6: Evaluate residuals
        std::cout << "\n6. Evaluating residuals..." << std::endl;
        
        factor->evaluate_residuals();
        Mat31 residual = factor->get_residual();
        
        std::cout << "Residual: " << residual.transpose() << std::endl;
        
        // Test 7: Evaluate Jacobians
        std::cout << "\n7. Evaluating Jacobians..." << std::endl;
        
        factor->evaluate_jacobians();
        Mat<3,9> jacobian = factor->get_jacobian(0);
        
        std::cout << "Jacobian:\n" << jacobian << std::endl;
        
        // Test 8: Evaluate chi2
        std::cout << "\n8. Evaluating chi2..." << std::endl;
        
        factor->evaluate_chi2();
        double chi2 = factor->get_chi2();
        
        std::cout << "Chi2 error: " << chi2 << std::endl;
        
        // Test 9: Print factor details
        std::cout << "\n9. Factor details:" << std::endl;
        factor->print();
        
        // Test 10: Test graph evaluation
        std::cout << "\n10. Testing graph evaluation..." << std::endl;
        
        // Evaluate the factor we just added
        std::cout << "Evaluating factor ID: " << factor->get_id() << std::endl;
        factor->evaluate_residuals();
        factor->evaluate_jacobians();
        factor->evaluate_chi2();
        
        std::cout << "Factor residual: " << factor->get_residual().transpose() << std::endl;
        std::cout << "Factor chi2: " << factor->get_chi2() << std::endl;
        
        // Test 11: Test with different observation times
        std::cout << "\n11. Testing with different observation times..." << std::endl;
        
        for (double t = 0.0; t <= 1.0; t += 0.2) {
            Mat51 obs_t = observation;
            obs_t(4) = t;  // Set observation time
            
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
