ivw_module(OpenTensorVisTTK)

set(HEADER_FILES
    include/inviwo/opentensorvisttk/opentensorvisttkmodule.h
    include/inviwo/opentensorvisttk/opentensorvisttkmoduledefine.h
    include/inviwo/opentensorvisttk/processors/opentensorvisttktestprocessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/opentensorvisttkmodule.cpp
    src/processors/opentensorvisttktestprocessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/opentensorvisttk-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})


include("C:/Users/jocja84/install/ttk/lib/cmake/ttkVTK/TTKVTKConfig.cmake")
target_link_libraries(inviwo-module-opentensorvisttk PUBLIC ttk::vtk::ttkAll)
target_include_directories(inviwo-module-opentensorvisttk PUBLIC "C:/Users/jocja84/install/ttk/vtk")
# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
