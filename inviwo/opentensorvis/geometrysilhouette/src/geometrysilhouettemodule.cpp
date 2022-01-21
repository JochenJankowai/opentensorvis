/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2022 Inviwo Foundation
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

#include <inviwo/geometrysilhouette/geometrysilhouettemodule.h>
#include <inviwo/geometrysilhouette/processors/geometrysilhouette.h>
#include <modules/opengl/shader/shadermanager.h>

namespace inviwo {

GeometrySilhouetteModule::GeometrySilhouetteModule(InviwoApplication* app) : InviwoModule(app, "GeometrySilhouette") {
    // Add a directory to the search path of the Shadermanager
    ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    // Register objects that can be shared with the rest of inviwo here:

    // Processors
    registerProcessor<GeometrySilhouette>();

    // Properties
    // registerProperty<GeometrySilhouetteProperty>();

    // Readers and writes
    // registerDataReader(std::make_unique<GeometrySilhouetteReader>());
    // registerDataWriter(std::make_unique<GeometrySilhouetteWriter>());

    // Data converters
    // registerRepresentationConverter(std::make_unique<GeometrySilhouetteDisk2RAMConverter>());

    // Ports
    // registerPort<GeometrySilhouetteOutport>();
    // registerPort<GeometrySilhouetteInport>();

    // PropertyWidgets
    // registerPropertyWidget<GeometrySilhouettePropertyWidget, GeometrySilhouetteProperty>("Default");

    // Dialogs
    // registerDialog<GeometrySilhouetteDialog>(GeometrySilhouetteOutport);

    // Other things
    // registerCapabilities(std::make_unique<GeometrySilhouetteCapabilities>());
    // registerSettings(std::make_unique<GeometrySilhouetteSettings>());
    // registerMetaData(std::make_unique<GeometrySilhouetteMetaData>());
    // registerPortInspector("GeometrySilhouetteOutport", "path/workspace.inv");
    // registerProcessorWidget(std::string processorClassName, std::unique_ptr<ProcessorWidget> processorWidget); 
    // registerDrawer(util::make_unique_ptr<GeometrySilhouetteDrawer>());
}

}  // namespace inviwo
