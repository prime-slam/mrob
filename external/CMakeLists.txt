add_subdirectory(Eigen)

if (BUILD_TESTING)
    add_subdirectory(Catch2)
endif()


set(PYBIND11_FINDPYTHON NEW)
if (NOT ANDROID)
    add_subdirectory(pybind11)
endif()
