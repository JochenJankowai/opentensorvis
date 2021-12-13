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

#include <inviwo/contourtree/processors/contourtreesimplificationprocessor.h>

#include "HyperVolume.h"
#include "Persistence.h"

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeSimplificationProcessor::processorInfo_{
    "org.inviwo.ContourTreeSimplificationProcessor",  // Class identifier
    "Contour Tree Simplification",                    // Display name
    "OpenTensorVis",                                  // Category
    CodeState::Experimental,                          // Code state
    "topology, merge tree, join tree, split tree",    // Tags
};
const ProcessorInfo ContourTreeSimplificationProcessor::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeSimplificationProcessor::ContourTreeSimplificationProcessor()
    : Processor()
    , contourTreeInport_("contourTreeInport")
    , contourTreeDataInport_("contourTreeDataInport")
    , contourTreeSimplificationOutport_("contourTreeSimplificationOutport")
    , simplificationMetod_("simplificationMetod", "Simplification method",
                           {{"persistence", "Persistence", SimplificationMetod::Persistence},
                            {"hypervolume", "Hypervolume", SimplificationMetod::Hypervolume}},
                           1) {
    addPorts(contourTreeInport_, contourTreeDataInport_, contourTreeSimplificationOutport_);

    addProperties(simplificationMetod_);
}

void ContourTreeSimplificationProcessor::process() {
    if (!contourTreeDataInport_.hasData() || !contourTreeDataInport_.getData()) return;

    const auto contourTreeData = contourTreeDataInport_.getData();

    auto simplifyCt = std::make_shared<contourtree::SimplifyCT>();

    simplifyCt->setInput(contourTreeData);

    std::shared_ptr<contourtree::SimFunction> simFn;

    if (simplificationMetod_.get() == SimplificationMetod::Persistence) {
        simFn = std::make_shared<contourtree::Persistence>(contourTreeData);
    } else {
        simFn = std::make_shared<contourtree::HyperVolume>(contourTreeData,
                                                           contourTreeInport_.getData()->arcMap);
    }

    simplifyCt->simplify(simFn);
    simplifyCt->computeWeights();

    contourTreeSimplificationOutport_.setData(simplifyCt);
}

}  // namespace inviwo
