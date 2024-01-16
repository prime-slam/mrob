# locate the additional necessary dependencies, if any

# extra source files
SET(sources
    SO3.cpp
    SE3.cpp
    SE3cov.cpp
    SE3vel.cpp
    SE3velCov.cpp
)

# extra header files
SET(headers
    mrob/SO3.hpp
    mrob/SE3.hpp
    mrob/SE3cov.hpp
    mrob/SE3vel.hpp
    mrob/SE3velCov.hpp
)

# create library
ADD_LIBRARY(SE3 ${sources})

SET_TARGET_PROPERTIES(SE3 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

# build tests if enabled
IF(BUILD_TESTING)
    ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTING)
