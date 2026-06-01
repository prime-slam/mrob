/* Test program for FactorTwistMatchingGravityBias
 *
 * This program tests the FactorTwistMatching class functionality
 * by creating test nodes and factors, then evaluating residuals and Jacobians.
 */

 #include <iostream>
 #include <memory>
 #include <Eigen/Dense>
 
 // Include the necessary mrob headers
 #include "mrob/factors/factor2Poses3dTwistMatchingBiasGravity.hpp"
#include "mrob/factors/factor2Poses3dTwistMatching.hpp"
 #include "mrob/factors/nodePose3dVelTc.hpp"
#include "mrob/factors/node3d.hpp"
#include "mrob/factors/nodeGravity3d.hpp"
 #include "mrob/matrix_base.hpp"
 #include "mrob/factor_graph.hpp"

 using namespace mrob;

 int main() {
    std::cout << "=== FactorTwistMatching Test Program ===" << std::endl;

    try {
        // Test 1: Create test data
        std::cout << "\n1. Creating test data..." << std::endl;
        
        // Create observation: [omegax, omegay, omegaz, accx, accy, accz] - IMU observation
        Mat61 observation;
        observation << 0.1, 0.2, 0.05,   // angular velocity [rad/s]
                       0.0, 0.0, 9.81;   // acceleration [m/s^2]
        
        // Information matrix for IMU measurements
        Mat9 obsInf = Mat9::Identity() * 100.0; // High confidence measurements
        
        std::cout << "IMU observation (omega, acc): " << observation.transpose() << std::endl;
        std::cout << "Information matrix diagonal: " << obsInf.diagonal().transpose() << std::endl;
        
        // Test 2: Create test nodes
        std::cout << "\n2. Creating test nodes..." << std::endl;
        
        // Create initial poses for origin and target nodes (SE3tc format: 5x5 matrix)
        Mat5 state_origin;
        state_origin << 1.0, 0.0, 0.0, 0.0, 0.0,  // Identity rotation + zero translation
                       0.0, 1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0,  // velocity and time
                        0.0, 0.0, 0.0, 0.5, 1.0;  // time = 0.5
        
        Mat5 state_target;
        state_target << 0.998, -0.052, 0.0, 0.1, 0.0,   // Small rotation + translation
                        0.052, 0.998, 0.0, 0.05, 0.0,
                        0.0, 0.0, 1.0, 0.01, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0, 1.0;    // time = 1.0
        
        // Create bias nodes (3D vectors)
        Mat31 bias_acc_init;
        bias_acc_init << 0.01, 0.02, -0.05;  // Accelerometer bias
        
        Mat31 bias_gyro_init; 
        bias_gyro_init << 0.001, -0.002, 0.0015; // Gyroscope bias
        
        // Create gravity node (3D vector pointing down)
        Mat31 gravity_init;
        gravity_init << 0.0, 0.0, -9.81; // Standard gravity
        
        std::cout << "Created bias acc: " << bias_acc_init.transpose() << std::endl;
        std::cout << "Created bias gyro: " << bias_gyro_init.transpose() << std::endl;
        std::cout << "Created gravity: " << gravity_init.transpose() << std::endl;
        
        // Test 3: Setup factor graph and nodes
        std::cout << "\n3. Setting up factor graph..." << std::endl;
        
        FGraph graph;
        
        // Create shared pointers to nodes
        std::shared_ptr<Node> nodeOrigin = std::make_shared<NodePose3dVelTc>(state_origin);
        std::shared_ptr<Node> nodeTarget = std::make_shared<NodePose3dVelTc>(state_target);
        std::shared_ptr<Node> nodeBiasAcc = std::make_shared<Node3d>(bias_acc_init);
        std::shared_ptr<Node> nodeBiasGyro = std::make_shared<Node3d>(bias_gyro_init);
        std::shared_ptr<Node> nodeGravity = std::make_shared<NodeGravity3d>(gravity_init);
        
        // Add nodes to graph
        factor_id_t origin_id = graph.add_node(nodeOrigin);
        factor_id_t target_id = graph.add_node(nodeTarget);
        factor_id_t bias_acc_id = graph.add_node(nodeBiasAcc);
        factor_id_t bias_gyro_id = graph.add_node(nodeBiasGyro);
        factor_id_t gravity_id = graph.add_node(nodeGravity);
        
        std::cout << "Node IDs - Origin: " << origin_id << ", Target: " << target_id << std::endl;
        std::cout << "Bias nodes - Acc: " << bias_acc_id << ", Gyro: " << bias_gyro_id << ", Gravity: " << gravity_id << std::endl;
        
        // Test 4: Create Factor with Bias and Gravity
        std::cout << "\n4. Creating FactorTwistMatchingBiasGravity..." << std::endl;
        
        auto factorBiasGravity = std::make_shared<Factor2Poses3dTwistMatchingBiasGravity>(
            observation, nodeOrigin, nodeTarget, nodeBiasAcc, nodeBiasGyro, nodeGravity,
            obsInf, Factor::robustFactorType::QUADRATIC
        );
        
        std::cout << "FactorTwistMatchingBiasGravity created successfully!" << std::endl;
        
        // Test 5: Evaluate residuals for bias+gravity factor
        std::cout << "\n5. Evaluating residuals for bias+gravity factor..." << std::endl;
        
        factorBiasGravity->evaluate_residuals();
        Mat91 residual_bg = factorBiasGravity->get_residual();
        
        std::cout << "Residual (bias+gravity): " << residual_bg.transpose() << std::endl;
        std::cout << "Residual norm: " << residual_bg.norm() << std::endl;
        
        // Test 6: Evaluate Jacobians for bias+gravity factor
        std::cout << "\n6. Evaluating Jacobians for bias+gravity factor..." << std::endl;
        
        factorBiasGravity->evaluate_jacobians();
        Mat<9,27> jacobian_bg = factorBiasGravity->get_jacobian(0);
        
        std::cout << "Jacobian shape: " << jacobian_bg.rows() << "x" << jacobian_bg.cols() << std::endl;
        std::cout << "Jacobian norm: " << jacobian_bg.norm() << std::endl;
        
        // Test 7: Evaluate chi2 for bias+gravity factor
        std::cout << "\n7. Evaluating chi2 for bias+gravity factor..." << std::endl;
        
        factorBiasGravity->evaluate_chi2();
        double chi2_bg = factorBiasGravity->get_chi2();
        
        std::cout << "Chi2 error (bias+gravity): " << chi2_bg << std::endl;
        
        // Test 8: Factor details
        std::cout << "\n8. Factor details (bias+gravity):" << std::endl;
        factorBiasGravity->print();
        
        // Test 9: Test simpler TwistMatching factor (without bias/gravity)
        std::cout << "\n9. Testing simpler TwistMatching factor..." << std::endl;
        
        auto factorSimple = std::make_shared<Factor2Poses3dTwistMatching>(
            observation, nodeOrigin, nodeTarget, obsInf, Factor::robustFactorType::QUADRATIC
        );
        
        factorSimple->evaluate_residuals();
        factorSimple->evaluate_jacobians();
        factorSimple->evaluate_chi2();
        
        Mat91 residual_simple = factorSimple->get_residual();
        Mat<9,18> jacobian_simple = factorSimple->get_jacobian(0);
        double chi2_simple = factorSimple->get_chi2();
        
        std::cout << "Simple factor residual: " << residual_simple.transpose() << std::endl;
        std::cout << "Simple factor residual norm: " << residual_simple.norm() << std::endl;
        std::cout << "Simple factor Jacobian shape: " << jacobian_simple.rows() << "x" << jacobian_simple.cols() << std::endl;
        std::cout << "Simple factor chi2: " << chi2_simple << std::endl;
        
        // Test 10: Test with different time intervals
        std::cout << "\n10. Testing with different time intervals..." << std::endl;
        
        for (double dt = 0.1; dt <= 1.0; dt += 0.2) {
            Mat5 state_target_dt = state_target;
            state_target_dt(4,4) = state_origin(4,4) + dt; // Adjust target time
            
            std::shared_ptr<Node> nodeTarget_dt = std::make_shared<NodePose3dVelTc>(state_target_dt);
            
            auto factor_dt = std::make_shared<Factor2Poses3dTwistMatching>(
                observation, nodeOrigin, nodeTarget_dt, obsInf, Factor::robustFactorType::QUADRATIC
            );
            
            factor_dt->evaluate_residuals();
            factor_dt->evaluate_chi2();
            
            Mat91 res_dt = factor_dt->get_residual();
            double chi2_dt = factor_dt->get_chi2();
            
            std::cout << "dt = " << dt << ": residual norm = " << res_dt.norm() 
                      << ", chi2 = " << chi2_dt << std::endl;
        }
        
        // Test 11: Test robustness with different observations
        std::cout << "\n11. Testing with different IMU observations..." << std::endl;
        
        std::vector<Mat61> test_observations = {
            (Mat61() << 0.0, 0.0, 0.0, 0.0, 0.0, 9.81).finished(),        // Zero motion
            (Mat61() << 1.0, 0.0, 0.0, 0.0, 0.0, 9.81).finished(),        // Pure rotation
            (Mat61() << 0.0, 0.0, 0.0, 1.0, 0.0, 9.81).finished(),        // Pure acceleration
            (Mat61() << 0.5, 0.3, 0.1, 0.2, 0.1, 9.81).finished()         // Mixed motion
        };
        
        std::vector<std::string> test_names = {"Zero motion", "Pure rotation", "Pure acceleration", "Mixed motion"};
        
        for (size_t i = 0; i < test_observations.size(); ++i) {
            auto factor_test = std::make_shared<Factor2Poses3dTwistMatching>(
                test_observations[i], nodeOrigin, nodeTarget, obsInf, Factor::robustFactorType::QUADRATIC
            );
            
            factor_test->evaluate_residuals();
            factor_test->evaluate_chi2();
            
            Mat91 res_test = factor_test->get_residual();
            double chi2_test = factor_test->get_chi2();
            
            std::cout << test_names[i] << ": residual norm = " << res_test.norm() 
                      << ", chi2 = " << chi2_test << std::endl;
        }
        
        std::cout << "\n=== All tests completed successfully! ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred!" << std::endl;
        return 1;
    }
    
    return 0;
 }
