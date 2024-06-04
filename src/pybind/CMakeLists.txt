if (NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "PYTHON_EXECUTABLE was not found in top level file")
endif ()

pybind11_add_module(pybind)

#####################################
# Set mrob python-package related variables
#####################################
set(PYTHON_BINARY_DIR ${MROB_LIB_DIR}/pybind)
set(PYTHON_BINARY_DST ${PROJECT_SOURCE_DIR}/mrobpy/mrob)

target_sources(pybind PRIVATE 
    mrobPy.cpp
    SE3py.cpp 
    PCRegistrationPy.cpp 
    PCPlanesPy.cpp 
    FGraphPy.cpp
    FGraphDiffPy.cpp
)

#####################################
# Configure target properties
#####################################
target_link_libraries(pybind PRIVATE 
    SE3 
    PCRegistration 
    FGraph
    FGraphDiff 
    plane-surfaces 
    visual
)

file(REMOVE_RECURSE ${PYTHON_BINARY_DIR})

set_target_properties(pybind PROPERTIES
    FOLDER "${PYTHON_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${PYTHON_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYTHON_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PYTHON_BINARY_DIR}"
)

#####################################
# Create Python package
#####################################
add_custom_target(python-package
    COMMAND ${CMAKE_COMMAND}
    -DPYTHON_BINARY_DIR=${PYTHON_BINARY_DIR}
    -DPYTHON_BINARY_DST=${PYTHON_BINARY_DST}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/make_python_package.cmake
    DEPENDS pybind
)

# add_custom_target(build-wheel
#     COMMAND ${PYTHON_EXECUTABLE} -m build --wheel --outdir ${PYTHON_WHEEL_DST_DIR}
#     COMMAND echo "pip wheel created at ${PYTHON_WHEEL_DST_DIR}"
#     WORKING_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}
#     DEPENDS python-package
# )