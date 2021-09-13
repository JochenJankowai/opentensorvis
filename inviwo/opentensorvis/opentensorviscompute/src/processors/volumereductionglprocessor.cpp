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

#include <inviwo/opentensorviscompute/processors/volumereductionglprocessor.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeReductionGLProcessor::processorInfo_{
    "org.inviwo.VolumeReductionGLProcessor",  // Class identifier
    "Volume Reduction Processor",             // Display name
    "OpenTensorVis",                          // Category
    CodeState::Experimental,                  // Code state
    Tags::GL,                                 // Tags
};
const ProcessorInfo VolumeReductionGLProcessor::getProcessorInfo() const { return processorInfo_; }

VolumeReductionGLProcessor::VolumeReductionGLProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , volumeOutport_("volumeOutport")
    , reductionOperator_("", "",
                         {{"min", "Min", ReductionOperator::Min},
                          {"max", "Max", ReductionOperator::Max},
                          {"sum", "Sum", ReductionOperator::Sum}}) {

    addPorts(volumeInport_, volumeOutport_);

    addProperties(reductionOperator_);
}

void VolumeReductionGLProcessor::process() {
    const auto reduced = gpuReduction_.reduce(volumeInport_.getData(), reductionOperator_.get());

    const auto val = reduced->getRepresentation<VolumeRAM>()->getAsDouble(size3_t{0});

    LogInfo(fmt::format("Reduced value: {}", val));

    volumeOutport_.setData(reduced);
}

}  // namespace inviwo
