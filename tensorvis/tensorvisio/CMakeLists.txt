#--------------------------------------------------------------------
# Inviwo TensorVisIO Module
ivw_module(TensorVisIO)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/modules/tensorvisio/processors/amiratensorreader.h
    include/modules/tensorvisio/processors/nrrdreader.h
    include/modules/tensorvisio/processors/tensorfield2dexport.h
    include/modules/tensorvisio/processors/tensorfield2dimport.h
    include/modules/tensorvisio/processors/tensorfield2dtovtk.h
    include/modules/tensorvisio/processors/tensorfield3dexport.h
    include/modules/tensorvisio/processors/tensorfield3dimport.h
<<<<<<< HEAD
=======
    include/modules/tensorvisio/processors/vtkdatasettotensorfield3d.h
<<<<<<< HEAD
    include/modules/tensorvisio/processors/vtkdatasetinformation.h
    include/modules/tensorvisio/processors/vtkreader.h
<<<<<<< HEAD
>>>>>>> 9f90ef3... Generic VTK reader: Basic version working
    include/modules/tensorvisio/processors/vtkrectilineargridreader.h
    include/modules/tensorvisio/processors/vtkstructuredgridreader.h
    include/modules/tensorvisio/processors/vtkstructuredpointsreader.h
    include/modules/tensorvisio/processors/vtktotensorfield3d.h
    include/modules/tensorvisio/processors/vtkunstructuredgridtorectilineargrid.h
    include/modules/tensorvisio/processors/vtkvolumereader.h
    include/modules/tensorvisio/processors/vtkxmlrectilineargridreader.h
    include/modules/tensorvisio/processors/witofilereader.h
=======
    include/modules/tensorvisio/processors/vtkunstructuredgridtorectilineargrid.h
    include/modules/tensorvisio/processors/vtkwriter.h
=======
>>>>>>> eedd932... Move VTK-only code to VTK module
    include/modules/tensorvisio/processors/flowguifilereader.h
>>>>>>> 66b7c02... Basic VTK pipeline in place
    include/modules/tensorvisio/tensorvisiomodule.h
    include/modules/tensorvisio/tensorvisiomoduledefine.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/processors/amiratensorreader.cpp
    src/processors/nrrdreader.cpp
    src/processors/tensorfield2dexport.cpp
    src/processors/tensorfield2dimport.cpp
    src/processors/tensorfield2dtovtk.cpp
    src/processors/tensorfield3dexport.cpp
    src/processors/tensorfield3dimport.cpp
<<<<<<< HEAD
=======
    src/processors/vtkdatasettotensorfield3d.cpp
<<<<<<< HEAD
    src/processors/vtkdatasetinformation.cpp
    src/processors/vtkreader.cpp
<<<<<<< HEAD
>>>>>>> 9f90ef3... Generic VTK reader: Basic version working
    src/processors/vtkrectilineargridreader.cpp
    src/processors/vtkstructuredgridreader.cpp
    src/processors/vtkstructuredpointsreader.cpp
    src/processors/vtktotensorfield3d.cpp
    src/processors/vtkunstructuredgridtorectilineargrid.cpp
    src/processors/vtkvolumereader.cpp
    src/processors/vtkxmlrectilineargridreader.cpp
    src/processors/witofilereader.cpp
=======
    src/processors/vtkunstructuredgridtorectilineargrid.cpp
    src/processors/vtkwriter.cpp
=======
>>>>>>> eedd932... Move VTK-only code to VTK module
    src/processors/flowguifilereader.cpp
>>>>>>> 66b7c02... Basic VTK pipeline in place
    src/tensorvisiomodule.cpp
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
	#${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/tensorvisio-test.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
# ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
