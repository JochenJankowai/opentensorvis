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

#include <inviwo/contourtree/processors/contourtreetopologicalfeaturesprocessor.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeTopologicalFeaturesProcessor::processorInfo_{
    "org.inviwo.ContourTreeTopologicalFeaturesProcessor",  // Class identifier
    "Contour Tree Topological Features Processor",         // Display name
    "OpenTensorVis",                                       // Category
    CodeState::Experimental,                               // Code state
    "topology, merge tree, join tree, split tree",         // Tags
};
const ProcessorInfo ContourTreeTopologicalFeaturesProcessor::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeTopologicalFeaturesProcessor::ContourTreeTopologicalFeaturesProcessor()
    : Processor()
    , contourTreeDataInport_("contourTreeDataInport")
    , contourTreeSimplificationInport_("contourTreeSimplificationInport")
    , contourTreeTopologicalFeatuesOutport_("contourTreeTopologicalFeatuesOutport")
    , featureType_("featureType", "Feature type",
                   {{"arc", "Arc", FeatureType::Arc},
                    {"partitionedExtrema", "Partitioned extrema", FeatureType::PartitionedExtrema}},
                   1) {

    addPorts(contourTreeDataInport_, contourTreeSimplificationInport_,
             contourTreeTopologicalFeatuesOutport_);

    addProperties(featureType_);
}

void ContourTreeTopologicalFeaturesProcessor::process() {
    if (!contourTreeDataInport_.hasData() || !contourTreeDataInport_.getData()) return;
    if (!contourTreeSimplificationInport_.hasData() || !contourTreeSimplificationInport_.getData())
        return;

    const auto contourTreeData = contourTreeDataInport_.getData();
    const auto contourTreeSimplification = contourTreeSimplificationInport_.getData();

    const auto partition = featureType_.get() == FeatureType::PartitionedExtrema;

    auto topologicalFeatures = std::make_shared<contourtree::TopologicalFeatures>();

    topologicalFeatures->loadDataFromArrays(contourTreeData, contourTreeSimplification->order,
                                            contourTreeSimplification->weights, partition);

    contourTreeTopologicalFeatuesOutport_.setData(topologicalFeatures);
}

}  // namespace inviwo
