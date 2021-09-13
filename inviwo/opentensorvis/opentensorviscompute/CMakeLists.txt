ivw_module(OpenTensorVisCompute)

set(HEADER_FILES
    include/inviwo/opentensorviscompute/algorithm/volumechannelsplitgl.h
    include/inviwo/opentensorviscompute/algorithm/volumeminmaxgl.h
    include/inviwo/opentensorviscompute/algorithm/volumenormalizationgl.h
    include/inviwo/opentensorviscompute/algorithm/volumereductiongl.h
    include/inviwo/opentensorviscompute/opentensorviscomputemodule.h
    include/inviwo/opentensorviscompute/opentensorviscomputemoduledefine.h
    include/inviwo/opentensorviscompute/processors/volumechannelsplitglprocessor.h
    include/inviwo/opentensorviscompute/processors/volumeminmaxglprocessor.h
    include/inviwo/opentensorviscompute/processors/volumenormalizationglprocessor.h
    include/inviwo/opentensorviscompute/processors/volumereductionglprocessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/algorithm/volumechannelsplitgl.cpp
    src/algorithm/volumeminmaxgl.cpp
    src/algorithm/volumenormalizationgl.cpp
    src/algorithm/volumereductiongl.cpp
    src/opentensorviscomputemodule.cpp
    src/processors/volumechannelsplitglprocessor.cpp
    src/processors/volumeminmaxglprocessor.cpp
    src/processors/volumenormalizationglprocessor.cpp
    src/processors/volumereductionglprocessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/glsl/volumesplit.comp
	${CMAKE_CURRENT_SOURCE_DIR}/glsl/volumereduction.comp
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
