# locate the necessary dependencies, if any


# extra source files
SET(sources
    plane.cpp
    plane_registration.cpp
    create_points.cpp
    estimate_plane.cpp
    factors/nodePlane4d.cpp
    factors/factor1Pose1Plane4d.cpp
    factors/EigenFactorPlane.cpp
    factors/EigenFactorPlaneCenter.cpp
)

# extra header files
SET(headers
    mrob/plane.hpp
    mrob/plane_registration.hpp
    mrob/create_points.hpp
    mrob/estimate_plane.hpp
    mrob/factors/nodePlane4d.hpp
    mrob/factors/factor1Pose1plane4d.hpp
    mrob/factors/EigenFactorPlane.hpp
    mrob/factors/EigenFactorPlaneCenter.hpp
)

# create the shared library
ADD_LIBRARY(plane-surfaces SHARED  ${sources} )
TARGET_LINK_LIBRARIES(plane-surfaces PCRegistration SE3 FGraph common)
# linking PCregistration is only due to plane_registration, remove when not needed


#ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
