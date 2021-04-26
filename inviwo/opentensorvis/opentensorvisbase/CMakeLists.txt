#--------------------------------------------------------------------
# Inviwo OpenTensorVisBase Module
ivw_module(OpenTensorVisBase)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/inviwo/opentensorvisbase/algorithm/tensorfieldsampling.h
    include/inviwo/opentensorvisbase/algorithm/tensorfieldslicing.h
    include/inviwo/opentensorvisbase/datastructures/attributes.h
    include/inviwo/opentensorvisbase/datastructures/attributes.inl
    include/inviwo/opentensorvisbase/datastructures/hyperstreamlinetracer.h
    include/inviwo/opentensorvisbase/datastructures/tensorfield.h
    include/inviwo/opentensorvisbase/datastructures/tensorfield2d.h
    include/inviwo/opentensorvisbase/datastructures/tensorfield3d.h
    include/inviwo/opentensorvisbase/datavisualizer/anisotropyraycastingvisualizer.h
    include/inviwo/opentensorvisbase/datavisualizer/hyperlicvisualizer2d.h
    include/inviwo/opentensorvisbase/datavisualizer/hyperlicvisualizer3d.h
    include/inviwo/opentensorvisbase/ports/tensorfieldport.h
    include/inviwo/opentensorvisbase/processors/hyperstreamlines.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dmetadata.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dsubsample.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dsubset.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dtoimage.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dfiberangle.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dbasismanipulation.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dboundingbox.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dinformation.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dmasktovolume.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dmetadata.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dsubsample.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dsubset.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dtodataframe.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dtovolume.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dgenerator.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dlic.h
    include/inviwo/opentensorvisbase/processors/tensorfield3dslice.h
    include/inviwo/opentensorvisbase/processors/tensorfield2dasrgba.h
    include/inviwo/opentensorvisbase/processors/volumeactualdataandvaluerange.h
    include/inviwo/opentensorvisbase/properties/eigenvalueproperty.h
    include/inviwo/opentensorvisbase/opentensorvisbasemodule.h
    include/inviwo/opentensorvisbase/opentensorvisbasemoduledefine.h
    include/inviwo/opentensorvisbase/util/attributeutil.h
    include/inviwo/opentensorvisbase/util/distancemetrics.h
    include/inviwo/opentensorvisbase/util/misc.h
    include/inviwo/opentensorvisbase/util/ndim.h
    include/inviwo/opentensorvisbase/util/setoperations.h
    include/inviwo/opentensorvisbase/util/tensorfieldutil.h
    include/inviwo/opentensorvisbase/util/tensorutil.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/algorithm/tensorfieldsampling.cpp
    src/algorithm/tensorfieldslicing.cpp
    src/datastructures/hyperstreamlinetracer.cpp
    src/datastructures/tensorfield2d.cpp
    src/datastructures/tensorfield3d.cpp
    src/datavisualizer/anisotropyraycastingvisualizer.cpp
    src/datavisualizer/hyperlicvisualizer2d.cpp
    src/datavisualizer/hyperlicvisualizer3d.cpp
    src/processors/hyperstreamlines.cpp
    src/processors/tensorfield2dmetadata.cpp
    src/processors/tensorfield2dsubsample.cpp
    src/processors/tensorfield2dsubset.cpp
    src/processors/tensorfield2dtoimage.cpp
    src/processors/tensorfield3dfiberangle.cpp
    src/processors/tensorfield3dbasismanipulation.cpp
    src/processors/tensorfield3dboundingbox.cpp
    src/processors/tensorfield3dinformation.cpp
    src/processors/tensorfield3dmasktovolume.cpp
    src/processors/tensorfield3dmetadata.cpp
    src/processors/tensorfield3dsubsample.cpp
    src/processors/tensorfield3dsubset.cpp
    src/processors/tensorfield3dtodataframe.cpp
    src/processors/tensorfield3dtovolume.cpp
    src/processors/tensorfield2dgenerator.cpp
    src/processors/tensorfield2dlic.cpp
    src/processors/tensorfield3dslice.cpp
    src/processors/tensorfield2dasrgba.cpp
    src/processors/volumeactualdataandvaluerange.cpp
    src/properties/eigenvalueproperty.cpp
    src/opentensorvisbasemodule.cpp
    src/util/tensorfieldutil.cpp
    src/util/tensorutil.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})


#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/eigen.glsl
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/eigenvaluefield.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/magnitudefield.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/quadrender.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/quadrender.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/tensorfieldtorgba.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/tensorlic2d.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/tensorutil.glsl
)
ivw_group("Shader Files" ${SHADER_FILES})


#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/opentensorvisbase-unittest-main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/arithmic-operations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/de_normalization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/distance-measures.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/set-operations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/to-string.cpp
)
ivw_add_unittest(${TEST_FILES})

if (NOT OPENMP_FOUND)
    find_package(OpenMP QUIET)
    if (OPENMP_FOUND)
        MESSAGE(STATUS "OpenMP flag set in OpenTensorVisBase module.")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()


#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Package or build shaders into resources
ivw_handle_shader_resources(${CMAKE_CURRENT_SOURCE_DIR}/glsl ${SHADER_FILES})
