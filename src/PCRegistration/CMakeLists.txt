# locate the necessary dependencies, if any


# extra source files
SET(sources
    arun.cpp
    gicp.cpp
    weight_point.cpp
    factors/factor1PosePoint2Point.cpp
    factors/factor1PosePoint2Plane.cpp
)

# extra header files
SET(headers
    mrob/pc_registration.hpp
    mrob/factors/factor1PosePoint2Point.hpp
    mrob/factors/factor1PosePoint2Plane.hpp
)

# create library
ADD_LIBRARY(PCRegistration ${sources} )
TARGET_LINK_LIBRARIES(PCRegistration SE3 FGraph common)

SET_TARGET_PROPERTIES(PCRegistration PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
