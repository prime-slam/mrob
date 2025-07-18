# locate the necessary dependencies, if any


SET(factors_headers
    mrob/factors/nodeSO3.hpp
    mrob/factors/factor1SO3obs.hpp
)

SET(factors_sources
    factors/nodeSO3.cpp
    factors/factor1SO3obs.cpp
)

# create library
ADD_LIBRARY(Attitude ${factors_sources})
TARGET_LINK_LIBRARIES(Attitude FGraph SE3 common)

SET_TARGET_PROPERTIES(Attitude PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

