# locate the necessary dependencies, if any


# extra source files
SET(sources
    plane.cpp
    plane_registration.cpp
    create_points.cpp
    estimate_plane.cpp
    utils_lie_differentiation.cpp
    factors/nodePlane4d.cpp
    factors/factor1Pose1Plane4d.cpp
    factors/EigenFactorPlane.cpp
    factors/EigenFactorPlaneCenter.cpp
    factors/EigenFactorPlaneRaw.cpp
    factors/EigenFactorPoint.cpp
    factors/PiFactorPlane.cpp
    factors/EigenFactorPlaneCoordinatesAlign.cpp
    factors/EigenFactorPlaneBase.cpp
    factors/EigenFactorPlaneDense.cpp
    factors/BaregEFPlane.cpp
)


# extra header files
SET(headers
    mrob/plane.hpp
    mrob/plane_registration.hpp
    mrob/create_points.hpp
    mrob/estimate_plane.hpp
    mrob/utils_lie_differentiation.hpp
    mrob/factors/nodePlane4d.hpp
    mrob/factors/factor1Pose1plane4d.hpp
    mrob/factors/EigenFactorPlane.hpp
    mrob/factors/EigenFactorPlaneCenter.hpp
    mrob/factors/EigenFactorPlaneRaw.hpp
    mrob/factors/EigenFactorPoint.hpp
    mrob/factors/PiFactorPlane.hpp
    mrob/factors/EigenFactorPlaneCoordinatesAlign.hpp
    mrob/factors/EigenFactorBase.hpp
    mrob/factors/EigenFactorDense.hpp
    mrob/factors/BaregEFPlane.hpp
)

# create library
ADD_LIBRARY(plane-surfaces ${sources} )
# linking PCregistration is only due to plane_registration, remove when not needed
TARGET_LINK_LIBRARIES(plane-surfaces PCRegistration SE3 FGraph common)

SET_TARGET_PROPERTIES(plane-surfaces PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

#ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
