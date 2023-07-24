# Clear DST dir from pybind
file(GLOB PYBIND_ARTIFACTS ${PYTHON_BINARY_DST}/pybind*)
message(STATUS "Last pybind artifact: ${PYBIND_ARTIFACTS}")
file(REMOVE "${PYBIND_ARTIFACTS}")

# Copy Python-bindings
file(INSTALL ${PYTHON_BINARY_DIR}/
        DESTINATION ${PYTHON_BINARY_DST}/)