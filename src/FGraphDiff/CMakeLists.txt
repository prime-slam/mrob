# locate the necessary dependencies, if any

project(FGraphDiff)

# extra header files
SET(headers
    ../FGraph/mrob/node.hpp
    mrob/factor_diff.hpp
    mrob/factor_graph_diff.hpp
    mrob/factor_graph_diff_solve.hpp
)

# extra source files
SET(sources
    ../FGraph/node.cpp
    factor_diff.cpp
    factor_graph_diff.cpp
    factor_graph_diff_solve.cpp
)

SET(factors_headers
    mrob/factors/factor1Pose2d_diff.hpp
    mrob/factors/factor2Poses2d_diff.hpp
)

SET(factors_sources
    factors/factor1Pose2d_diff.cpp
    factors/factor2Poses2d_diff.cpp
)

# create library
ADD_LIBRARY(FGraphDiff ${sources} ${factors_sources})
TARGET_LINK_LIBRARIES(FGraphDiff SE3 common FGraph)

SET_TARGET_PROPERTIES(FGraphDiff PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(tests)
