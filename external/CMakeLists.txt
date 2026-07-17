add_subdirectory(Eigen)

if (BUILD_TESTING)
    add_subdirectory(Catch2)
endif()

if (BUILD_PYTHON_BINDINGS)
    add_subdirectory(pybind11)
endif()
