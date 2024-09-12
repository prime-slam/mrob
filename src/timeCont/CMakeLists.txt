# locate the necessary dependencies, if any

# extra header files
#SET(headers
#)

# extra source files
#SET(sources
#    factor_graph_solve.cpp
#)

SET(factors_headers
    mrob/factors/factor2Poses3dTwistMatching.hpp
)

SET(factors_sources
    factors/factor2Poses3dTwistMatching.cpp
)

# create library
ADD_LIBRARY(FGraphTime ${sources} ${factors_sources})
TARGET_LINK_LIBRARIES(FGraphTime FGraph SE3 common)

SET_TARGET_PROPERTIES(FGraphTime PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

