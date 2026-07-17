#####################################
# Standalone mode
#####################################
# This directory builds in two ways:
#  - as part of the full mrob build (default developer flow), or
#  - standalone, linking a prebuilt mrob core build dir passed via -DMROB_CORE_BUILD_DIR.
#    The wheel scripts in tools/ use this to compile the Python-independent C++ core once
#    and then build only these bindings per Python version, each in a fresh build dir so
#    no FindPython state can leak from one version to the next.
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    project(mrob-python LANGUAGES CXX)

    if (NOT MROB_CORE_BUILD_DIR)
        message(FATAL_ERROR "Standalone bindings build: pass -DMROB_CORE_BUILD_DIR=<mrob core build dir> (a build of the top-level project with -DBUILD_PYTHON_BINDINGS=OFF)")
    endif ()
    if (NOT EXISTS ${MROB_CORE_BUILD_DIR}/mrobTargets.cmake)
        message(FATAL_ERROR "${MROB_CORE_BUILD_DIR}/mrobTargets.cmake not found -- build the core first")
    endif ()

    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release")
    endif ()
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_EXTENSIONS ON)

    # Same PYTHON_EXECUTABLE -> FindPython bridge as the top-level CMakeLists
    if (PYTHON_EXECUTABLE)
        if (Python_EXECUTABLE AND NOT Python_EXECUTABLE STREQUAL PYTHON_EXECUTABLE)
            message(FATAL_ERROR "This build directory was configured for Python at '${Python_EXECUTABLE}'. "
                    "Reconfiguring it for a different interpreter is not supported (CMake's FindPython "
                    "caches its detection) -- use a fresh build directory instead.")
        endif ()
        set(Python_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "Path to the Python interpreter")
    endif ()
    set(PYBIND11_FINDPYTHON NEW)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../external/pybind11 external/pybind11)

    # Import the prebuilt core libraries as mrob:: targets
    include(${MROB_CORE_BUILD_DIR}/mrobTargets.cmake)

    set(MROB_LIB_DIR ${CMAKE_BINARY_DIR}/lib)
endif ()

pybind11_add_module(pybind)

#####################################
# Set mrob python-package related variables
#####################################
get_filename_component(MROB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
set(PYTHON_BINARY_DIR ${MROB_LIB_DIR}/pybind)
set(PYTHON_BINARY_DST ${MROB_ROOT_DIR}/mrobpy/mrob)

target_sources(pybind PRIVATE 
    mrobPy.cpp
    SE3py.cpp 
    PCRegistrationPy.cpp 
    PCPlanesPy.cpp 
    FGraphPy.cpp
    FGraphTimePy.cpp
)

#####################################
# Configure target properties
#####################################
target_link_libraries(pybind PRIVATE 
    mrob::SE3 
    mrob::PCRegistration 
    mrob::FGraph 
    mrob::plane-surfaces 
    mrob::visual
    mrob::FGraphTime
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
