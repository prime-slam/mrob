IF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") #TODO: /Wall but disable noisy warnings
    ADD_COMPILE_DEFINITIONS(_USE_MATH_DEFINES)
    SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror=return-type")
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override -Werror=return-type")
ELSE()
ENDIF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")

ADD_SUBDIRECTORY(common)

ADD_SUBDIRECTORY(geometry)

ADD_SUBDIRECTORY(FGraph)

ADD_SUBDIRECTORY(PCRegistration)

ADD_SUBDIRECTORY(plane-surfaces)

ADD_SUBDIRECTORY(visual)

ADD_SUBDIRECTORY(timeCont)


# Namespaced aliases so consumers (the pybind module below, or a standalone bindings
# build importing mrobTargets.cmake) always link mrob::<lib> regardless of which mode
# they are built in.
FOREACH(lib common SE3 FGraph PCRegistration plane-surfaces visual FGraphTime)
    ADD_LIBRARY(mrob::${lib} ALIAS ${lib})
ENDFOREACH()

IF(BUILD_PYTHON_BINDINGS)
    ADD_SUBDIRECTORY(pybind)
ENDIF(BUILD_PYTHON_BINDINGS)

# Build-tree export: allows a separate build tree (src/pybind standalone, used by the
# wheel scripts in tools/) to link the libraries compiled here without rebuilding them.
# The C++ core is Python-independent, so the wheel pipeline compiles it once and reuses
# it for every Python version. eigen is exported too because the core public headers
# expose Eigen types, so its include paths are part of the link interface.
EXPORT(TARGETS common SE3 FGraph PCRegistration plane-surfaces visual FGraphTime eigen
       NAMESPACE mrob:: FILE ${PROJECT_BINARY_DIR}/mrobTargets.cmake)
