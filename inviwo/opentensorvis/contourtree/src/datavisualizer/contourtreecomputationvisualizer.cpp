/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018-2020 Inviwo Foundation
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

#include <inviwo/contourtree/datavisualizer/contourtreecomputationvisualizer.h>

#include <inviwo/core/processors/processorutils.h>
#include <inviwo/core/io/datareaderfactory.h>
#include <inviwo/core/ports/volumeport.h>

#include <inviwo/contourtree/processors/contourtreecomputationprocessor.h>
#include <inviwo/contourtree/processors/contourtreedataprocessor.h>
#include <inviwo/contourtree/processors/contourtreesimplificationprocessor.h>
#include <inviwo/contourtree/processors/contourtreetopologicalfeaturesprocessor.h>
#include <inviwo/contourtree/processors/contourtreequeryprocessor.h>

namespace inviwo {

using GP = util::GridPos;

ContourTreeComputationVisualizer::ContourTreeComputationVisualizer(InviwoApplication* app)
    : DataVisualizer{}, app_(app) {}

std::string ContourTreeComputationVisualizer::getName() const {
    return "Compute and query a contour tree";
}

Document ContourTreeComputationVisualizer::getDescription() const {
    Document doc;
    auto b = doc.append("html").append("body");
    b.append("", "Compute and query a contour tree");
    return doc;
}

std::vector<FileExtension> ContourTreeComputationVisualizer::getSupportedFileExtensions() const {
    return std::vector<FileExtension>{};
}

bool ContourTreeComputationVisualizer::isOutportSupported(const Outport* port) const {
    return dynamic_cast<const VolumeOutport*>(port) != nullptr;
}

bool ContourTreeComputationVisualizer::hasSourceProcessor() const { return false; }

bool ContourTreeComputationVisualizer::hasVisualizerNetwork() const { return true; }

std::pair<Processor*, Outport*> ContourTreeComputationVisualizer::addSourceProcessor(
    const std::string&, ProcessorNetwork*) const {
    return {nullptr, nullptr};
}

std::vector<Processor*> ContourTreeComputationVisualizer::addVisualizerNetwork(
    Outport* outport, ProcessorNetwork* net) const {

    auto contourTreeComputation =
        net->addProcessor(util::makeProcessor<ContourTreeComputationProcessor>(GP{0, 0}));
    auto contourTreeData =
        net->addProcessor(util::makeProcessor<ContourTreeDataProcessor>(GP{2, 3}));
    auto contourTreeSimplification =
        net->addProcessor(util::makeProcessor<ContourTreeSimplificationProcessor>(GP{2, 6}));
    auto topologicalFeatures =
        net->addProcessor(util::makeProcessor<ContourTreeTopologicalFeaturesProcessor>(GP{2, 9}));
    auto contourTreeQuery =
        net->addProcessor(util::makeProcessor<ContourTreeQueryProcessor>(GP{0, 12}));

    net->addConnection(contourTreeComputation->getOutports()[0], contourTreeData->getInports()[0]);

    net->addConnection(contourTreeData->getOutports()[0],
                       contourTreeSimplification->getInports()[0]);

    net->addConnection(contourTreeData->getOutports()[1],
                       contourTreeSimplification->getInports()[1]);

    net->addConnection(contourTreeSimplification->getOutports()[0],
                       topologicalFeatures->getInports()[0]);

    net->addConnection(contourTreeSimplification->getOutports()[1],
                       topologicalFeatures->getInports()[1]);

    net->addConnection(contourTreeComputation->getOutports()[0], contourTreeQuery->getInports()[0]);
    net->addConnection(contourTreeComputation->getOutports()[1], contourTreeQuery->getInports()[1]);
    net->addConnection(topologicalFeatures->getOutports()[0], contourTreeQuery->getInports()[2]);
    net->addConnection(topologicalFeatures->getOutports()[1], contourTreeQuery->getInports()[3]);
    net->addConnection(topologicalFeatures->getOutports()[2], contourTreeQuery->getInports()[4]);

    net->addConnection(outport, contourTreeComputation->getInports()[0]);

    return {contourTreeComputation, contourTreeData, contourTreeSimplification, topologicalFeatures,
            contourTreeQuery};
}

std::vector<Processor*> ContourTreeComputationVisualizer::addSourceAndVisualizerNetwork(
    const std::string&, ProcessorNetwork*) const {

    return {nullptr};
}

}  // namespace inviwo
