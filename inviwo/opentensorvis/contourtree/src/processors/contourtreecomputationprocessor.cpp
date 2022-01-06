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

#include <inviwo/contourtree/processors/contourtreecomputationprocessor.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <Grid3D.h>
#include <MergeTree.h>
#include <inviwo/core/network/networklock.h>
#include <tuple>
#include <execution>
#include <inviwo/opentensorviscompute/algorithm/volumereductiongl.h>
#include <inviwo/core/datastructures/geometry/typedmesh.h>
#include <inviwo/contourtree/util/util.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeComputationProcessor::processorInfo_{
    "org.inviwo.ContourTreeComputationProcessor",   // Class identifier
    "Contour Tree Computation",                     // Display name
    "OpenTensorVis",                                // Category
    CodeState::Experimental,                        // Code state
    "topology, merge tree, join tree, split tree",  // Tags
};
const ProcessorInfo ContourTreeComputationProcessor::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeComputationProcessor::ContourTreeComputationProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , volumeOutport_("passthrough")
    , contourTreeOutport_("contourTreeOutport")
    , treeType_("treeType", "Tree type",
                {{"join", "Join", contourtree::TreeType::JoinTree},
                 {"split", "Split", contourtree::TreeType::SplitTree}},
                0) {
    addPorts(volumeInport_, volumeOutport_,contourTreeOutport_);
    addProperties(treeType_);
}

void ContourTreeComputationProcessor::process() {
    if (!util::checkPorts(volumeInport_)) return;

    auto inputVolume = volumeInport_.getData();

    if (inputVolume->getDataFormat()->getComponents() != 1) {
        LogWarn("Volume has more than one channel. Aborting.");
        return;
    }

    inputVolume->getRepresentation<VolumeRAM>()->dispatch<void, dispatching::filter::Scalars>(
        [&, this](auto vrprecision) {
            using ValueType = util::PrecisionValueType<decltype(vrprecision)>;

            const auto dimensions = inputVolume->getDimensions();

            auto grid = std::make_shared<contourtree::Grid3D<ValueType>>(
                static_cast<int>(dimensions.x), static_cast<int>(dimensions.y),
                static_cast<int>(dimensions.z));

            grid->fnVals.resize(glm::compMul(dimensions));
            auto gridData = grid->fnVals.data();

            auto volumeData = vrprecision->getDataTyped();

            std::memcpy(gridData, volumeData, glm::compMul(dimensions) * sizeof(ValueType));

            const auto treeType = treeType_.get();

            auto mergeTree = std::make_shared<contourtree::MergeTree>();

            mergeTree->computeTree(grid, treeType);

            mergeTree->generateArrays(treeType);

            contourTreeOutport_.setData(mergeTree);
        });

    volumeOutport_.setData(volumeInport_.getData());
}

}  // namespace inviwo
