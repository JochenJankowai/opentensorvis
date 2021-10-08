#--------------------------------------------------------------------
# Inviwo ContourTree Module
ivw_module(ContourTree)

#--------------------------------------------------------------------
# Build libContourTree
add_subdirectory(ext/contour-tree)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/inviwo/contourtree/contourtreemodule.h
    include/inviwo/contourtree/contourtreemoduledefine.h
    include/inviwo/contourtree/processors/contourtreeprocessor.h
    include/inviwo/contourtree/processors/volumelabelselectionprocessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/contourtreemodule.cpp
    src/processors/contourtreeprocessor.cpp
    src/processors/volumelabelselectionprocessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
)
ivw_group("Shader Files" ${SHADER_FILES})

#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
    tests/unittests/contourtree-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

ivw_make_package(InviwoContourTreeModule inviwo-module-contourtree)

target_link_libraries(inviwo-module-contourtree PUBLIC harishcontourtree)
