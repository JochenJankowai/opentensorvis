ivw_module(VolumeJacobian)

set(HEADER_FILES
    include/inviwo/volumejacobian/volumejacobianmodule.h
    include/inviwo/volumejacobian/volumejacobianmoduledefine.h
    include/inviwo/volumejacobian/processors/volumejacobianoperations.h
    include/inviwo/volumejacobian/util/volumejacobian.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/volumejacobianmodule.cpp
    src/processors/volumejacobianoperations.cpp
    src/util/volumejacobian.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/volumejacobian-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
