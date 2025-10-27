# locate the necessary dependencies, if any


SET(factors_headers
    mrob/factors/nodeSO3.hpp
    mrob/factors/factor1SO3obs.hpp
    mrob/factors/factor_gyro_prop.hpp
    mrob/factors/factor_gravity_align.hpp
    mrob/factors/factor_gyro_bias_prop.hpp
    mrob/factors/factor_rotated_gyro_bias_prop.hpp
)

SET(factors_sources
    factors/nodeSO3.cpp
    factors/factor1SO3obs.cpp
    factors/factor_gyro_prop.cpp
    factors/factor_gravity_align.cpp
    factors/factor_gyro_bias_prop.cpp
    factors/factor_rotated_gyro_bias_prop.cpp
)

# create library
ADD_LIBRARY(Attitude ${factors_sources})
TARGET_LINK_LIBRARIES(Attitude FGraph SE3 common)

SET_TARGET_PROPERTIES(Attitude PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

