# locate the necessary dependencies, if any


# extra source files
SET(sources
    factors/factorCameraProj3dPoint.cpp
    factors/factorCameraProj3dLine.cpp
    #factors/nodeCameraPinHole.cpp
)

# extra header files
SET(headers
    mrob/factors/factorCameraProj3dPoint.hpp
    mrob/factors/factorCameraProj3dLine.hpp
    #mrob/factors/nodeCameraPinHole.hpp
)

# create library
ADD_LIBRARY(visual ${sources} )
TARGET_LINK_LIBRARIES(visual SE3 FGraph common)

SET_TARGET_PROPERTIES(visual PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MROB_LIB_DIR}"
)

#ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
