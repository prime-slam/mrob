# locate the necessary dependencies, if any


SET(factors_headers
    mrob/factors/factor2Poses3dTwistMatching.hpp
    mrob/factors/factor2Poses3dTwistMatchingBiasGravity.hpp
    mrob/factors/factor1Pose3dVel.hpp
    mrob/factors/nodePose3dVelTc.hpp
    mrob/factors/node3d.hpp
    mrob/factors/nodeGravity3d.hpp
    mrob/factors/factorLidarMap2PoseInterpolation.hpp
    mrob/factors/factorLidarMap2PoseInterpolationJacobian.hpp
    mrob/factors/factorPriorInertial.hpp
)

SET(factors_sources
    factors/factor2Poses3dTwistMatching.cpp
    factors/factor2Poses3dTwistMatchingBiasGravity.cpp
    factors/nodePose3dVelTc.cpp
    factors/factor1Pose3dVel.cpp
    factors/node3d.cpp
    factors/nodeGravity3d.cpp
    factors/factorLidarMap2PoseInterpolation.cpp
    factors/factorLidarMap2PoseInterpolationJacobian.cpp
    factors/factorPriorInertial.cpp
)

# create library
ADD_LIBRARY(FGraphTime ${factors_sources})
TARGET_LINK_LIBRARIES(FGraphTime FGraph SE3 common)

SET_TARGET_PROPERTIES(FGraphTime PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

# Add examples subdirectory
add_subdirectory(examples)

