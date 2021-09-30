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

#include <modules/contourtree/processors/contourtree.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <Grid3D.hpp>
#include <MergeTree.hpp>
#include <ContourTreeData.hpp>
#include <SimplifyCT.hpp>
#include <Persistence.hpp>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTree::processorInfo_{
    "org.inviwo.ContourTree",  // Class identifier
    "Contour Tree",            // Display name
    "Undefined",               // Category
    CodeState::Experimental,   // Code state
    Tags::None,                // Tags
};
const ProcessorInfo ContourTree::getProcessorInfo() const { return processorInfo_; }

ContourTree::ContourTree()
    : Processor()
    , volumeInport_("volumeInport")
    , fieldOutport_("scalar_field")
    , segmentationOutport_("segmentation")
    , simplification_("simplification", "Simplification Threshold", 0.0f, 0.0f, 1.0f) {

    addPorts(volumeInport_, fieldOutport_, segmentationOutport_);

    addProperties(simplification_);
}

void ContourTree::process() {
    auto inputVolume = volumeInport_.getData();

    if (inputVolume->getDataFormat()->getComponents() != 1) {
        LogWarn("Volume has more than once channel. Aborting.");
        return;
    }

    auto seg =
        inputVolume->getRepresentation<VolumeRAM>()
            ->dispatch<std::shared_ptr<Volume>, dispatching::filter::Scalars>(
                [&](auto vrprecision) {
                    using ValueType = util::PrecisionValueType<decltype(vrprecision)>;

                    const auto dimensions = inputVolume->getDimensions();

                    contourtree::Grid3D<ValueType> grid(static_cast<int>(dimensions.x),
                                                        static_cast<int>(dimensions.y),
                                                        static_cast<int>(dimensions.z));

                    grid.fnVals.resize(glm::compMul(dimensions));
                    auto gridData = grid.fnVals.data();

                    auto volumeData = vrprecision->getDataTyped();

                    std::memcpy(gridData, volumeData, glm::compMul(dimensions) * sizeof(ValueType));

                    contourtree::MergeTree contourTree;

                    const auto treeType = contourtree::TreeType::ContourTree;

                    contourTree.computeTree(&grid, treeType);

                    contourtree::ContourTreeData ctdata(contourTree);

                    contourtree::SimplifyCT sim;
                    sim.setInput(&ctdata);

                    auto simFn = new contourtree::Persistence(ctdata);

                    sim.simplify(simFn);

                    return std::make_shared<Volume>();
                });

    segmentationOutport_.setData(seg);
}

}  // namespace inviwo
