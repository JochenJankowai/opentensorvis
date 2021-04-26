#--------------------------------------------------------------------
# Inviwo OpenTensorVisIO Module
ivw_module(OpenTensorVisIO)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/inviwo/opentensorvisio/processors/amiratensorreader.h
    include/inviwo/opentensorvisio/processors/flowguifilereader.h
    include/inviwo/opentensorvisio/processors/nrrdreader.h
    include/inviwo/opentensorvisio/processors/tensorfield2dexport.h
    include/inviwo/opentensorvisio/processors/tensorfield2dimport.h
    include/inviwo/opentensorvisio/processors/tensorfield3dexport.h
    include/inviwo/opentensorvisio/processors/tensorfield3dimport.h
    include/inviwo/opentensorvisio/opentensorvisiomodule.h
    include/inviwo/opentensorvisio/opentensorvisiomoduledefine.h
    include/inviwo/opentensorvisio/util/util.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/processors/amiratensorreader.cpp
    src/processors/flowguifilereader.cpp
    src/processors/nrrdreader.cpp
    src/processors/tensorfield2dexport.cpp
    src/processors/tensorfield2dimport.cpp
    src/processors/tensorfield3dexport.cpp
    src/processors/tensorfield3dimport.cpp
    src/opentensorvisiomodule.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/glsl/tensorfieldioprocessor.frag
)
ivw_group("Shader Files" ${SHADER_FILES})

#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
	#${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/opentensorvisio-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
# ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
