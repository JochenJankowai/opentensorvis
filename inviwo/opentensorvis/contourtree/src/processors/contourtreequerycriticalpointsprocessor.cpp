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

#include <inviwo/contourtree/processors/contourtreequerycriticalpointsprocessor.h>
#include <inviwo/contourtree/util/util.h>
#include <inviwo/core/util/zip.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeQueryCriticalPointsProcessor::processorInfo_{
    "org.inviwo.ContourTreeQueryCriticalPointsProcessor",  // Class identifier
    "Contour Tree Query Critical Points Processor",        // Display name
    "OpenTensorVis",                                       // Category
    CodeState::Experimental,                               // Code state
    Tags::None,                                            // Tags
};
const ProcessorInfo ContourTreeQueryCriticalPointsProcessor::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeQueryCriticalPointsProcessor::ContourTreeQueryCriticalPointsProcessor()
    : Processor()
    , contourTreeSimplificationInport_("contourTreeSimplificationInport")
    , volumeOutport_("volumeOutport")
    , nLeaves_("nLeaves", "Number of leaves", 1, 1, 12, 1) {

    addPorts(contourTreeSimplificationInport_, volumeOutport_);
    addProperties(nLeaves_);
}

void ContourTreeQueryCriticalPointsProcessor::process() {
    if (!util::checkPorts(contourTreeSimplificationInport_)) return;

    const auto contourTreeSimplification = contourTreeSimplificationInport_.getData();

    size_t i{0};

    for (const auto [a, b, c] :
         util::zip(contourTreeSimplification->branches, *contourTreeSimplification->fn_,
                   contourTreeSimplification->fnv)) {
        if (i > 10) break;

        LogInfo(fmt::format("branch{},{},{}", i++, b, c));
    }
}

}  // namespace inviwo
