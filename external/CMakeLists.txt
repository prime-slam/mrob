add_subdirectory(Eigen)

if (BUILD_TESTING)
    add_subdirectory(Catch2)
endif()

if (NOT ANDROID)
    add_subdirectory(pybind11)
endif()
