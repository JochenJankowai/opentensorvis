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

#include <inviwo/contourtree/processors/ContourTreeProcessor.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <Grid3D.h>
#include <MergeTree.h>
#include <Persistence.h>
#include <TopologicalFeatures.h>
#include <inviwo/core/network/networklock.h>
#include <tuple>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeProcessor::processorInfo_{
    "org.inviwo.ContourTreeProcessor",  // Class identifier
    "Contour Tree",            // Display name
    "OpenTensorVis",           // Category
    CodeState::Experimental,   // Code state
    Tags::CPU,                 // Tags
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
    , simplificationCriterion_("simplificationCriterion", "Simplification criterion",
                               {{"topk", "Top k features", SimplificationCriterion::TopKFeatures},
                                {"threshold", "Threshold", SimplificationCriterion::Threshold}},
                               0)
    , topKFeatures_("topKFeatures", "Top k features", 1, 0)
    , threshold_("threshold", "Threshold", 0.0f, 0.0f, 1.0f, 0.0001f)
    , hasData_(false) {

    addPorts(volumeInport_, segmentationOutport_);

    topKFeatures_.visibilityDependsOn(simplificationCriterion_,
                                      [](TemplateOptionProperty<SimplificationCriterion>& p) {
                                          return p.get() == SimplificationCriterion::TopKFeatures;
                                      });

    threshold_.visibilityDependsOn(simplificationCriterion_,
                                   [](TemplateOptionProperty<SimplificationCriterion>& p) {
                                       return p.get() == SimplificationCriterion::Threshold;
                                   });

    addProperties(treeType_, featureType_, simplificationCriterion_, topKFeatures_, threshold_);

    treeType_.onChange([this]() { computeTree(); });
    volumeInport_.onChange([this]() {
        computeTree();
    });
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

                const auto simFn = new contourtree::Persistence(contourTreeData);

                simplifyCt.simplify(simFn);
                simplifyCt.computeWeights();

                return std::tuple(contourTreeData, simplifyCt, contourTree.arcMap);
            });

    contourTreeData_ = contourTreeData;
    simplifyCt_ = simplifyCt;
    arcMap_ = arcMap;

    hasData_ = true;
}

void ContourTreeProcessor::process() {
    if (!hasData_) return;

    const bool partition = featureType_.get() == FeatureType::Arc ? false : true;

    contourtree::TopologicalFeatures topologicalFeatures;
    topologicalFeatures.loadDataFromArrays(contourTreeData_, simplifyCt_.order, simplifyCt_.weights,
                                           partition);

    std::vector<contourtree::Feature> features;

    const int topKFeatures = simplificationCriterion_.get() == SimplificationCriterion::Threshold
                                 ? -1
                                 : topKFeatures_.get();

    if (featureType_.get() == FeatureType::Arc) {
        features = topologicalFeatures.getArcFeatures(topKFeatures, threshold_.get());
    }
    if (featureType_.get() == FeatureType::PartitionedExtrema) {
        features =
            topologicalFeatures.getPartitionedExtremaFeatures(topKFeatures, threshold_.get());
    }

    LogInfo(fmt::format("{} features selected.", features.size()));

    auto inputVolume = volumeInport_.getData();

    auto segmentationVolume =
        std::make_shared<Volume>(inputVolume->getDimensions(), DataUInt16::get());

    auto rawData = dynamic_cast<VolumeRAMPrecision<glm::u16>*>(
                       segmentationVolume->getEditableRepresentation<VolumeRAM>())
                       ->getDataTyped();

    /*
     * Look up which feature the arcId in arcMap belongs to and assign value. That should be it.
     */
    glm::u16 n{static_cast<glm::u16>(topKFeatures-1)};
    std::fill_n(rawData, glm::compMul(inputVolume->getDimensions()), topKFeatures);

    for (const auto& feature : features) {

        for (size_t i{0}; i < arcMap_.size(); ++i) {
            const auto arcID = arcMap_[i];
            auto& currentValue = rawData[i];

            for (const auto featureArcID : feature.arcs) {
                if (arcID == featureArcID) {
                    currentValue = std::min(currentValue, n);
                }
            }
        }

        n--;
    }

    LogInfo("Assigned ids:") std::unordered_set<glm::u16> s;
    for (int i{0}; i < glm::compMul(inputVolume->getDimensions()); ++i) {
        s.insert(rawData[i]);
    }
    for (auto val : s) {
        LogInfo(fmt::format("{}", val));
    }

    segmentationVolume->dataMap_.dataRange = segmentationVolume->dataMap_.valueRange = dvec2{0, topKFeatures};
    segmentationVolume->setBasis(inputVolume->getBasis());
    segmentationVolume->setOffset(inputVolume->getOffset());

    segmentationOutport_.setData(segmentationVolume);
}

}  // namespace inviwo
