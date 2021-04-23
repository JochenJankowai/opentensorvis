/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2018 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/tensorvisio/tensorvisiomodule.h>

<<<<<<< HEAD
#include <modules/tensorvisio/processors/amiratensorreader.h>
#include <modules/tensorvisio/processors/nrrdreader.h>
#include <modules/tensorvisio/processors/tensorfield2dexport.h>
#include <modules/tensorvisio/processors/tensorfield2dimport.h>
#include <modules/tensorvisio/processors/tensorfield2dtovtk.h>
#include <modules/tensorvisio/processors/tensorfield3dexport.h>
#include <modules/tensorvisio/processors/tensorfield3dimport.h>
#include <modules/tensorvisio/processors/vtkdatasettotensorfield3d.h>
<<<<<<< HEAD
#include <modules/tensorvisio/processors/vtkdatasetinformation.h>
#include <modules/tensorvisio/processors/vtkreader.h>
#include <modules/tensorvisio/processors/vtkunstructuredgridtorectilineargrid.h>
<<<<<<< HEAD
#include <modules/tensorvisio/processors/vtkstructuredgridreader.h>
#include <modules/tensorvisio/processors/vtktotensorfield3d.h>
#include <modules/tensorvisio/processors/witofilereader.h>
#include <modules/tensorvisio/processors/vtkrectilineargridreader.h>
#include <modules/tensorvisio/processors/vtkxmlrectilineargridreader.h>
#include <modules/tensorvisio/processors/vtkstructuredpointsreader.h>
#include <modules/tensorvisio/processors/vtkvolumereader.h>
#include <modules/tensorvisio/util/vtkoutputlogger.h>
=======
#include <modules/tensorvisio/processors/flowguifilereader.h>
<<<<<<< HEAD
#include <modules/tensorvisio/processors/vtkwriter.h>
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 66b7c02... Basic VTK pipeline in place
=======
#include <modules/tensorvisio/util/vtkoutputlogger.h>
>>>>>>> 57758cd... TensorVisIO: Added missing include
=======
>>>>>>> fa193bd... TensorVis/VTK: Moved VTK Output Logger to the VTK module
=======
#include <modules/tensorvisio/processors/flowguifilereader.h>
>>>>>>> eedd932... Move VTK-only code to VTK module
=======
#include <modules/tensorvisio/processors/vtktotensorfield2d.h>
>>>>>>> 60d78da... TensorVisIO: Added VTK to Tensor Field 2D processor
=======
#include <inviwo/tensorvisio/processors/amiratensorreader.h>
#include <inviwo/tensorvisio/processors/nrrdreader.h>
#include <inviwo/tensorvisio/processors/tensorfield2dexport.h>
#include <inviwo/tensorvisio/processors/tensorfield2dimport.h>
#include <inviwo/tensorvisio/processors/tensorfield2dtovtk.h>
#include <inviwo/tensorvisio/processors/tensorfield3dexport.h>
#include <inviwo/tensorvisio/processors/tensorfield3dimport.h>
#include <inviwo/tensorvisio/processors/vtkdatasettotensorfield3d.h>
#include <inviwo/tensorvisio/processors/flowguifilereader.h>
#include <inviwo/tensorvisio/processors/vtktotensorfield2d.h>
>>>>>>> ad0038d... TensorVis: Refactor include directory

namespace inviwo {

<<<<<<< HEAD
TensorVisIOModule::TensorVisIOModule(InviwoApplication* app)
    : InviwoModule{app, "TensorVisIO"}{

=======
TensorVisIOModule::TensorVisIOModule(InviwoApplication* app) : InviwoModule(app, "TensorVisIO") {
>>>>>>> 9f90ef3... Generic VTK reader: Basic version working
    registerProcessor<AmiraTensorReader>();
    registerProcessor<NRRDReader>();
    registerProcessor<TensorField2DExport>();
    registerProcessor<TensorField2DImport>();
    registerProcessor<TensorField2DToVTK>();
    registerProcessor<TensorField3DExport>();
    registerProcessor<TensorField3DImport>();
    registerProcessor<VTKDataSetToTensorField3D>();
    registerProcessor<FlowGUIFileReader>();
    registerProcessor<VTKToTensorField2D>();
}

<<<<<<< HEAD
TensorVisIOModule::~TensorVisIOModule() = default;

=======
>>>>>>> 9f90ef3... Generic VTK reader: Basic version working
}  // namespace inviwo
