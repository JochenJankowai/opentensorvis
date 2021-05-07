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

#include <inviwo/contourexplorer/processors/contourtreetosegmentationvolume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/util/indexmapper.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeToSegmentationVolume::processorInfo_{
    "org.inviwo.ContourTreeToSegmentationVolume",  // Class identifier
    "Contour Tree To Segmentation Volume",         // Display name
    "OpenTensorVis",                               // Category
    CodeState::Experimental,                       // Code state
    Tags::CPU,                                     // Tags
};
const ProcessorInfo ContourTreeToSegmentationVolume::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeToSegmentationVolume::ContourTreeToSegmentationVolume()
    : Processor()
    , contourTreeInport_("contourTreeInport")
    , volumeInport_("volumeInport")
    , volumeOutport_("volumeOutport") {

    addPorts(volumeInport_, contourTreeInport_, volumeOutport_);
}

void ContourTreeToSegmentationVolume::process() {
    auto contourTree = contourTreeInport_.getData()->tree;
    auto inputVolume = volumeInport_.getData();

    const auto dimensions = inputVolume->getDimensions();
    const auto numberOfVoxels = glm::compMul(dimensions);
    util::IndexMapper3D indexMapper(dimensions);

    auto outputVolume = std::make_shared<Volume>(dimensions, DataUInt8::get());
    outputVolume->setModelMatrix(inputVolume->getModelMatrix());
    outputVolume->setWorldMatrix(inputVolume->getWorldMatrix());
    outputVolume->setBasis(inputVolume->getBasis());
    outputVolume->setOffset(inputVolume->getOffset());

    std::vector<ttk::ftm::idSuperArc> ids(numberOfVoxels);

    for (ttk::SimplexId i{0}; i < static_cast<ttk::SimplexId>(numberOfVoxels); ++i) {
        ids[i] = contourTree->getCorrespondingSuperArcId(i);
    }

    std::vector<ttk::ftm::idSuperArc> uids(ids);

    std::unique(std::begin(uids), std::end(uids));

    std::map<ttk::ftm::idSuperArc, ttk::ftm::idSuperArc> mapping;

    for (size_t u{0}; u < uids.size(); ++u) {
        mapping.insert_or_assign(uids[u], u);
    }

    std::transform(std::begin(ids), std::end(ids), std::begin(ids),
                   [&mapping](const auto v) { return mapping[v]; });

    auto data = static_cast<glm::uint8_t*>(static_cast<VolumeRAMPrecision<glm::uint8_t>*>(
                                               outputVolume->getEditableRepresentation<VolumeRAM>())
                                               ->getData());

    std::copy(std::begin(ids), std::end(ids), data);

    volumeOutport_.setData(outputVolume);
}

}  // namespace inviwo
