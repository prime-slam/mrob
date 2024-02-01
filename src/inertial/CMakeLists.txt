# locate the necessary dependencies, if any


# extra source files
SET(sources
    factors/nodeInertial3d.cpp
    filter_lidar_inertial.cpp
)

# extra header files
SET(headers
    mrob/factors/nodeInertial3d.hpp
    mrob/filter_lidar_inertial.hpp
)

# create library
ADD_LIBRARY(inertial ${sources} )
TARGET_LINK_LIBRARIES(inertial SE3 FGraph common)

SET_TARGET_PROPERTIES(inertial PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

#ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
