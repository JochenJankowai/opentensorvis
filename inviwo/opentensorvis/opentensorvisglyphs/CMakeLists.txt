ivw_module(OpenTensorVisGlyphs)

set(HEADER_FILES
    include/inviwo/opentensorvisglyphs/opentensorvisglyphsmodule.h
    include/inviwo/opentensorvisglyphs/opentensorvisglyphsmoduledefine.h
    include/inviwo/opentensorvisglyphs/datastructures/deformablecube.h
    include/inviwo/opentensorvisglyphs/datastructures/deformablecylinder.h
    include/inviwo/opentensorvisglyphs/datastructures/deformablesphere.h
    include/inviwo/opentensorvisglyphs/processors/tensorglyphprocessor.h
    include/inviwo/opentensorvisglyphs/processors/tensorglyphrenderer.h
    include/inviwo/opentensorvisglyphs/properties/tensorglyphproperty.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/opentensorvisglyphsmodule.cpp
    src/datastructures/deformablecube.cpp
    src/datastructures/deformablecylinder.cpp
    src/datastructures/deformablesphere.cpp
    src/processors/tensorglyphprocessor.cpp
    src/processors/tensorglyphrenderer.cpp
    src/properties/tensorglyphproperty.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/tensorglyphpicking.frag
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/opentensorvisglyphs-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
