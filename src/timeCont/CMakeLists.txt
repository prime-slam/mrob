# locate the necessary dependencies, if any


SET(factors_headers
    mrob/factors/factor2Poses3dTwistMatching.hpp
    mrob/factors/nodePose3dVelTc.hpp
    mrob/factors/factor1Pose3dVel.hpp
)

SET(factors_sources
    factors/factor2Poses3dTwistMatching.cpp
    factors/nodePose3dVelTc.cpp
    factors/factor1Pose3dVel.cpp
)

# create library
ADD_LIBRARY(FGraphTime ${factors_sources})
TARGET_LINK_LIBRARIES(FGraphTime FGraph SE3 common)

SET_TARGET_PROPERTIES(FGraphTime PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

