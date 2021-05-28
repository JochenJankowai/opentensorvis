ivw_module(OpenTensorVisPlotting)

set(HEADER_FILES
    include/inviwo/opentensorvisplotting/opentensorvisplottingmodule.h
    include/inviwo/opentensorvisplotting/opentensorvisplottingmoduledefine.h
    include/inviwo/opentensorvisplotting/processors/lineplotprocessor.h
    include/inviwo/opentensorvisplotting/processors/tensorfield3dtodataframe.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/opentensorvisplottingmodule.cpp
    src/processors/lineplotprocessor.cpp
    src/processors/tensorfield3dtodataframe.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/opentensorvisplotting-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
