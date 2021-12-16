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

#include <inviwo/contourexplorer/processors/segmentationvolumetransferfunctionprocessor.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/opentensorviscompute/algorithm/volumereductiongl.h>
#include <inviwo/contourexplorer/algorithm/generatesegmentedtf.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SegmentationVolumeTransferFunctionProcessor::processorInfo_{
    "org.inviwo.SegmentationVolumeTransferFunctionProcessor",  // Class identifier
    "Segmentation Volume Transfer Function",                   // Display name
    "OpenTensorVis",                                           // Category
    CodeState::Experimental,                                   // Code state
    "tf, transfer function, cluster, segmentation",            // Tags
};
const ProcessorInfo SegmentationVolumeTransferFunctionProcessor::getProcessorInfo() const {
    return processorInfo_;
}

SegmentationVolumeTransferFunctionProcessor::SegmentationVolumeTransferFunctionProcessor()
    : Processor()
    , brushingAndLinkingInport_("brushingAndLinkingInport")
    , volumeInport_("volumeInport")
    , tfProperty_("tf", "Transfer function")
    , slope_("slope", "Slope", 0.1, 0.01, 0.5, 0.001)
    , shadeColor_("shadeColor", "Shade color", vec4(0), vec4(0), vec4(1), vec4(0.00001f),
                  InvalidationLevel::InvalidOutput, PropertySemantics::Color) {

    addPorts(brushingAndLinkingInport_, volumeInport_);

    addProperties(tfProperty_, slope_, shadeColor_);
}

void SegmentationVolumeTransferFunctionProcessor::process() {
    if (!volumeInport_.hasData() || !volumeInport_.getData()) return;

    const auto inputVolume = volumeInport_.getData();

    const auto maxValue = static_cast<int32_t>(inputVolume->dataMap_.dataRange.y);

    const auto numberOfFeatures = maxValue + 1;

    const auto selection = brushingAndLinkingInport_.getSelectedIndices();

    auto& tf = tfProperty_.get();

    const auto tfPoints = SegmentationTransferFunctionGenerator::generateTFPrimitivesForSegments(
        selection, numberOfFeatures, slope_.get(), shadeColor_.get());

    NetworkLock l;

    tf.clear();

    for (const auto& tfPrimitive : tfPoints) {
        tf.add(tfPrimitive);
    }
}

}  // namespace inviwo
