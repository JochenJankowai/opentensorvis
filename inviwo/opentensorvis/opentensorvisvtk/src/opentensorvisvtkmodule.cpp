/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2021 Inviwo Foundation
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

#include <inviwo/opentensorvisvtk/opentensorvisvtkmodule.h>

namespace inviwo {

OpenTensorVisVTKModule::OpenTensorVisVTKModule(InviwoApplication* app) : InviwoModule(app, "OpenTensorVisVTK") {
    // Add a directory to the search path of the Shadermanager
    // ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    // Register objects that can be shared with the rest of inviwo here:

    // Processors
    // registerProcessor<OpenTensorVisVTKProcessor>();

    // Properties
    // registerProperty<OpenTensorVisVTKProperty>();

    // Readers and writes
    // registerDataReader(std::make_unique<OpenTensorVisVTKReader>());
    // registerDataWriter(std::make_unique<OpenTensorVisVTKWriter>());

    // Data converters
    // registerRepresentationConverter(std::make_unique<OpenTensorVisVTKDisk2RAMConverter>());

    // Ports
    // registerPort<OpenTensorVisVTKOutport>();
    // registerPort<OpenTensorVisVTKInport>();

    // PropertyWidgets
    // registerPropertyWidget<OpenTensorVisVTKPropertyWidget, OpenTensorVisVTKProperty>("Default");

    // Dialogs
    // registerDialog<OpenTensorVisVTKDialog>(OpenTensorVisVTKOutport);

    // Other things
    // registerCapabilities(std::make_unique<OpenTensorVisVTKCapabilities>());
    // registerSettings(std::make_unique<OpenTensorVisVTKSettings>());
    // registerMetaData(std::make_unique<OpenTensorVisVTKMetaData>());
    // registerPortInspector("OpenTensorVisVTKOutport", "path/workspace.inv");
    // registerProcessorWidget(std::string processorClassName, std::unique_ptr<ProcessorWidget> processorWidget); 
    // registerDrawer(util::make_unique_ptr<OpenTensorVisVTKDrawer>());
}

}  // namespace inviwo
