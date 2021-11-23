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

#include <inviwo/contourtree/processors/contourtreeprocessor.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <Grid3D.h>
#include <MergeTree.h>
#include <SimFunction.h>
#include <Persistence.h>
#include <HyperVolume.h>
#include <inviwo/core/network/networklock.h>
#include <tuple>
#include <execution>
#include <inviwo/opentensorviscompute/algorithm/volumereductiongl.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeProcessor::processorInfo_{
    "org.inviwo.ContourTreeProcessor",              // Class identifier
    "Contour Tree",                                 // Display name
    "OpenTensorVis",                                // Category
    CodeState::Experimental,                        // Code state
    "topology, merge tree, join tree, split tree",  // Tags
};
const ProcessorInfo ContourTreeProcessor::getProcessorInfo() const { return processorInfo_; }

ContourTreeProcessor::ContourTreeProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , segmentationOutport_("segmentation")
    , treeType_("treeType", "Tree type",
                {{"join", "Join", contourtree::TreeType::JoinTree},
                 {"split", "Split", contourtree::TreeType::SplitTree},
                 {"contour", "Contour", contourtree::TreeType::ContourTree}},
                0)
    , featureType_("featureType", "Feature type",
                   {{"arc", "Arc", FeatureType::Arc},
                    {"partitionedExtrema", "Partitioned extrema", FeatureType::PartitionedExtrema}},
                   1)
    , queryCriterion_("simplificationCriterion", "Simplification criterion",
                      {{"topk", "Top k features", QueryCriterion::TopKFeatures},
                       {"threshold", "Threshold", QueryCriterion::Threshold}},
                      0)
    , simplificationMetod_("simplificationMetod", "Simplification method",
                           {{"persistence", "Persistence", SimplificationMetod::Persistence},
                            {"hypervolume", "Hypervolume", SimplificationMetod::Hypervolume}},1)
    , topKFeatures_("topKFeatures", "Top k features", 3, 1, 20)
    , threshold_("threshold", "Threshold", 0.0f, 0.0f, 1.0f, 0.0001f)
    , hasData_(false) {

    addPorts(volumeInport_, segmentationOutport_);

    topKFeatures_.visibilityDependsOn(queryCriterion_,
                                      [](TemplateOptionProperty<QueryCriterion>& p) {
                                          return p.get() == QueryCriterion::TopKFeatures;
                                      });

    threshold_.visibilityDependsOn(queryCriterion_, [](TemplateOptionProperty<QueryCriterion>& p) {
        return p.get() == QueryCriterion::Threshold;
    });

    featureType_.setReadOnly(true);

    addProperties(treeType_, featureType_, queryCriterion_, topKFeatures_,

    auto updateTree = [this]() { computeTree(); };

    treeType_.onChange(updateTree);
    simplificationMetod_.onChange(updateTree);
    volumeInport_.onChange(updateTree);
}

void ContourTreeProcessor::computeTree() {
    if (!volumeInport_.hasData() || !volumeInport_.getData()) {
        return;
    }

    auto inputVolume = volumeInport_.getData();

    if (inputVolume->getDataFormat()->getComponents() != 1) {
        LogWarn("Volume has more than one channel. Aborting.");
        return;
    }

    auto [contourTreeData, simplifyCt, arcMap] =
        inputVolume->getRepresentation<VolumeRAM>()
            ->dispatch<std::tuple<contourtree::ContourTreeData, contourtree::SimplifyCT,
                                  std::vector<uint32_t>>,
                       dispatching::filter::Scalars>([&](auto vrprecision) {
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

                const auto treeType = treeType_.get();

                contourTree.computeTree(&grid, treeType);

                contourTree.generateArrays(treeType);

                contourtree::ContourTreeData contourTreeData(contourTree);

                contourtree::SimplifyCT simplifyCt;

                simplifyCt.setInput(&contourTreeData);

                contourtree::SimFunction* simFn;

                if (simplificationMetod_.get() == SimplificationMetod::Persistence) {
                    simFn = new contourtree::Persistence(contourTreeData);
                } else {
                    simFn = new contourtree::HyperVolume(contourTreeData, contourTree.arcMap);
                }

                simplifyCt.simplify(simFn);
                simplifyCt.computeWeights();

                return std::tuple(contourTreeData, simplifyCt, contourTree.arcMap);
            });

    topologicalFeatures_.loadDataFromArrays(contourTreeData, simplifyCt.order, simplifyCt.weights,
                                           true);
    arcMap_ = std::move(arcMap);
    hasData_ = true;
}

void ContourTreeProcessor::process() {
    if (!hasData_) computeTree();
    
    const auto features =
        topologicalFeatures_.getPartitionedExtremaFeatures(topKFeatures_.get(), threshold_.get());

    const auto inputVolume = volumeInport_.getData();

    /*
     * Look up which feature the arcId in arcMap belongs to and assign value. That should be it.
     */
    const auto numberOfElements = glm::compMul(inputVolume->getDimensions());

    auto rawData = new uint16_t[numberOfElements];
    std::fill_n(rawData, numberOfElements, 0);

    for (size_t i{1}; i < features.size(); ++i) {
        for (const auto arcId : features[i].arcs) {
            for (size_t j{0}; j < numberOfElements; ++j) {
                if (arcMap_[j] == arcId) {
                    rawData[j] = static_cast<uint16_t>(i);
                }
            }
        }
    }

    auto printHistogram = [](uint16_t* data, size_t numberOfElements) {
        std::map<glm::u16, int> histogram;
        for (size_t i{0}; i < numberOfElements; ++i) {
            histogram[data[i]]++;
        }

        LogInfoCustom("ContourTreeProcessor", "Histogram");
        for (auto pair : histogram) {
            LogInfoCustom("ContourTreeProcessor", fmt::format("{0}: {1}", pair.first, pair.second));
        }
    };

    printHistogram(rawData, numberOfElements);

    const auto segmentationVolumeRamPrecision =
        std::make_shared<VolumeRAMPrecision<uint16_t>>(rawData, inputVolume->getDimensions());

    auto segmentationVolume = std::make_shared<Volume>(segmentationVolumeRamPrecision);

    segmentationVolume->dataMap_.dataRange = segmentationVolume->dataMap_.valueRange =
        dvec2{0, features.size() - 1};
    segmentationVolume->setBasis(inputVolume->getBasis());
    segmentationVolume->setOffset(inputVolume->getOffset());
    
    segmentationVolume->setInterpolation(InterpolationType::Nearest);

    segmentationOutport_.setData(segmentationVolume);
}

}  // namespace inviwo
