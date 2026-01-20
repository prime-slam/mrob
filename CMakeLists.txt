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
# External dependencies
#####################################
add_subdirectory(external)

#####################################
# Sources
#####################################
add_subdirectory(src)

