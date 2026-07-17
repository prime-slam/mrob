# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/time_profiling.hpp
    mrob/optimizer.hpp
)

# extra source files
SET(sources
    time_profiling.cpp
    optimizer.cpp
)
# create library
ADD_LIBRARY(common ${sources})
TARGET_INCLUDE_DIRECTORIES(common PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
TARGET_LINK_LIBRARIES(common PUBLIC Eigen3::Eigen)

SET_TARGET_PROPERTIES(common PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)