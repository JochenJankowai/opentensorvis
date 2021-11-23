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

#include <inviwo/contourexplorer/processors/criticalpointstomeshprocessor.h>
#include <modules/base/algorithm/meshutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CriticalPointsToMeshProcessor::processorInfo_{
    "org.inviwo.CriticalPointsToMeshProcessor",  // Class identifier
    "Critical Points To Mesh Processor",         // Display name
    "OpenTensorVis",                             // Category
    CodeState::Experimental,                     // Code state
    "mimima, maxima, critical, topology",        // Tags
};
const ProcessorInfo CriticalPointsToMeshProcessor::getProcessorInfo() const {
    return processorInfo_;
}

CriticalPointsToMeshProcessor::CriticalPointsToMeshProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , meshOutport_("meshOutport")
    , criticalPointType_(
          "criticalPointType", "Type", {{"minima", "Minima", CriticalPointType::Mimimum}, {"maxima", "Maxima", CriticalPointType::Maximum}})
    , numberOfCriticalPoints_("numberOfCriticalPoints","Number of critical points", 1,1,20,1) {

    addPorts(volumeInport_, meshOutport_);
    addProperties(criticalPointType_, numberOfCriticalPoints_);
}

void CriticalPointsToMeshProcessor::process() {
    auto inputVolume = volumeInport_.getData();

    auto positions =
        inputVolume->getRepresentation<VolumeRAM>()
            ->dispatch<std::vector<vec3>, dispatching::filter::Scalars>([&](auto vrprecision) {
                using ValueType = util::PrecisionValueType<decltype(vrprecision)>;


            });

    auto outputMesh = std::shared_ptr<BasicMesh>();

    

    for (const auto& position:positions) {
        auto mesh = meshutil::sphere(position, 0.2f, vec4(1));
        outputMesh->Mesh::append(*mesh);
    }


    
}

}  // namespace inviwo
