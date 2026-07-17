# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(mrob)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Release") 
ENDIF (NOT CMAKE_BUILD_TYPE)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
IF(APPLE)
    MESSAGE(STATUS "OSX Architecture: ${CMAKE_OSX_ARCHITECTURES}")
ENDIF(APPLE)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_EXTENSIONS ON)

if (WIN32)
    set(BUILD_SHARED_LIBS OFF)
else()
    option(BUILD_SHARED_LIBS "Build shared library" ON)
endif()

SET(MROB_LIB_DIR ${CMAKE_BINARY_DIR}/lib)

#####################################
# Testing option
#####################################
# By default tests are turned off but the value is cached and can be changed in Cmake-GUI
set(BUILD_TESTING false CACHE BOOL "Build unit tests for MROB modules")

MESSAGE(STATUS "Build Tests: ${BUILD_TESTING}")

IF(ANDROID)
    SET(BUILD_TESTING false)
ELSE(ANDROID)
    IF(BUILD_TESTING)
        enable_testing()
    ENDIF(BUILD_TESTING)
ENDIF(ANDROID)

#####################################
# Python bindings option
#####################################
option(BUILD_PYTHON_BINDINGS "Build the mrob Python bindings" ON)
IF(ANDROID)
    SET(BUILD_PYTHON_BINDINGS OFF)
ENDIF(ANDROID)

#####################################
# External dependencies
#####################################
if (BUILD_PYTHON_BINDINGS)
    # pybind11 v3 uses CMake's modern FindPython module, driven by the Python_EXECUTABLE
    # (capital P) variable, not the legacy PYTHON_EXECUTABLE this project's docs and
    # scripts use to pin the interpreter. Bridge it here, before Python is first searched
    # for (inside external/pybind11). Note: reconfiguring an existing build dir with a
    # different interpreter is not supported (FindPython caches its detection) -- use a
    # fresh build dir instead; the wheel scripts in tools/ already work that way.
    if (PYTHON_EXECUTABLE)
        if (Python_EXECUTABLE AND NOT Python_EXECUTABLE STREQUAL PYTHON_EXECUTABLE)
            message(FATAL_ERROR "This build directory was configured for Python at '${Python_EXECUTABLE}'. "
                    "Reconfiguring it for a different interpreter is not supported (CMake's FindPython "
                    "caches its detection) -- use a fresh build directory instead.")
        endif ()
        set(Python_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "Path to the Python interpreter")
    endif ()
    set(PYBIND11_FINDPYTHON NEW)
endif ()
add_subdirectory(external)

#####################################
# Sources
#####################################
add_subdirectory(src)

