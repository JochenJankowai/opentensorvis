ivw_module(OpenTensorVisCompute)

set(HEADER_FILES
    include/inviwo/opentensorviscompute/opentensorviscomputemodule.h
    include/inviwo/opentensorviscompute/opentensorviscomputemoduledefine.h
    include/inviwo/opentensorviscompute/algorithm/gpureduction.h
    include/inviwo/opentensorviscompute/algorithm/volumenormalization.h
    include/inviwo/opentensorviscompute/processors/volumenormalizationprocessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/opentensorviscomputemodule.cpp
    src/algorithm/gpureduction.cpp
    src/algorithm/volumenormalization.cpp
    src/processors/volumenormalizationprocessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/glsl/reduction.comp
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/volumenormalization.comp
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/opentensorviscompute-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
