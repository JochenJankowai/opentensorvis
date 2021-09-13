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

#include <inviwo/opentensorvisvtk/processors/volumetovtkunstructuredgridprocessor.h>
#include <inviwo/opentensorvisvtk/algorithm/volumetovtkunstructuredgrid.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeToVTKUnstructuredGridProcessor::processorInfo_{
    "org.inviwo.VolumeToVTKUnstructuredGridProcessor",  // Class identifier
    "Volume To VTK Unstructured Grid",                  // Display name
    "OpenTensorVis",                                    // Category
    CodeState::Experimental,                            // Code state
    Tags::CPU,                                          // Tags
};
const ProcessorInfo VolumeToVTKUnstructuredGridProcessor::getProcessorInfo() const {
    return processorInfo_;
}

VolumeToVTKUnstructuredGridProcessor::VolumeToVTKUnstructuredGridProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , vtkDataSetOutport_("vtkDataSetOutport")
    , name_("", "", "Volume data") {

    addPorts(volumeInport_, vtkDataSetOutport_);

    addProperties(name_);

    name_.onChange([this]() { invalidate(InvalidationLevel::InvalidOutput); });
}

void VolumeToVTKUnstructuredGridProcessor::process() {
    VolumeToVTKUnstructuredGrid converter;
    vtkDataSetOutport_.setData(
        std::make_shared<VTKDataSet>(converter.convert(volumeInport_.getData(), name_.get())));
}

}  // namespace inviwo
