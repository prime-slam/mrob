# Clear DST dir from pybind
file(GLOB PYBIND_ARTIFACTS ${PYTHON_BINARY_DST}/pybind*)
file(REMOVE ${PYBIND_ARTIFACTS})

# Copy Python-bindings
file(INSTALL ${PYTHON_BINARY_DIR}/
        DESTINATION ${PYTHON_BINARY_DST}/)