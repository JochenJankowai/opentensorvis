ivw_module(GeometrySilhouette)

set(HEADER_FILES
    include/inviwo/geometrysilhouette/geometrysilhouettemodule.h
    include/inviwo/geometrysilhouette/geometrysilhouettemoduledefine.h
    include/inviwo/geometrysilhouette/processors/geometrysilhouette.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/geometrysilhouettemodule.cpp
    src/processors/geometrysilhouette.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/geometrysilhouette.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/geometrysilhouette.frag
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/geometrysilhouette-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
