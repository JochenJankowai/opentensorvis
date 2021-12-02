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
    include/inviwo/contourtree/ports/contourtreedataport.h
    include/inviwo/contourtree/ports/contourtreeport.h
    include/inviwo/contourtree/ports/contourtreesimplificationport.h
    include/inviwo/contourtree/ports/contourtreetopologicalfeaturesport.h
    include/inviwo/contourtree/processors/contourtreecomputationprocessor.h
    include/inviwo/contourtree/processors/contourtreedataprocessor.h
    include/inviwo/contourtree/processors/contourtreequeryprocessor.h
    include/inviwo/contourtree/processors/contourtreesimplificationprocessor.h
    include/inviwo/contourtree/processors/contourtreetopologicalfeaturesprocessor.h
    include/inviwo/contourtree/util/util.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/contourtreemodule.cpp
    src/processors/contourtreecomputationprocessor.cpp
    src/processors/contourtreedataprocessor.cpp
    src/processors/contourtreequeryprocessor.cpp
    src/processors/contourtreesimplificationprocessor.cpp
    src/processors/contourtreetopologicalfeaturesprocessor.cpp
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
