ivw_module(OpenTensorVisVTK)

set(HEADER_FILES
    include/inviwo/opentensorvisvtk/algorithm/volumetovtkimagedata.h
    include/inviwo/opentensorvisvtk/algorithm/volumetovtkunstructuredgrid.h
    include/inviwo/opentensorvisvtk/opentensorvisvtkmodule.h
    include/inviwo/opentensorvisvtk/opentensorvisvtkmoduledefine.h
    include/inviwo/opentensorvisvtk/processors/tensorfield2dtovtk.h
    include/inviwo/opentensorvisvtk/processors/volumetovtkimagedataprocessor.h
    include/inviwo/opentensorvisvtk/processors/volumetovtkunstructuredgridprocessor.h
    include/inviwo/opentensorvisvtk/processors/vtkdatasettotensorfield2d.h
    include/inviwo/opentensorvisvtk/processors/vtkdatasettotensorfield3d.h
    include/inviwo/opentensorvisvtk/util/util.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/algorithm/volumetovtkimagedata.cpp
    src/algorithm/volumetovtkunstructuredgrid.cpp
    src/opentensorvisvtkmodule.cpp
    src/processors/tensorfield2dtovtk.cpp
    src/processors/volumetovtkimagedataprocessor.cpp
    src/processors/volumetovtkunstructuredgridprocessor.cpp
    src/processors/vtkdatasettotensorfield2d.cpp
    src/processors/vtkdatasettotensorfield3d.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/opentensorvisvtk-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
