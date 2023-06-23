# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/differential_factor_graph.hpp
    mrob/differential_factor.hpp
)

# extra source files
SET(sources
    differential_factor_graph.cpp
)

SET(factors_headers
    mrob/factors/diffFactor1PosePoint2Point.hpp
)

SET(factors_sources
    factors/diffFactor1PosePoint2Point.cpp
)

# create the shared library
ADD_LIBRARY(DiffFGraph  ${sources} ${factors_sources})
TARGET_LINK_LIBRARIES(DiffFGraph FGraph SE3 common)
