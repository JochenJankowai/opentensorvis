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
    , simplificationCriterion_("simplificationCriterion", "Simplification criterion",
                               {{"topk", "Top k features", SimplificationCriterion::TopKFeatures}/*,
                                {"threshold", "Threshold", SimplificationCriterion::Threshold}*/},
                               0)
    , topKFeatures_("topKFeatures", "Top k features", 3, 1, 20)
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
    volumeInport_.onChange([this]() { computeTree(); });
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
    if (!hasData_) computeTree();

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

    

    auto inputVolume = volumeInport_.getData();

    auto segmentationVolume =
        std::make_shared<Volume>(inputVolume->getDimensions(), DataUInt16::get());

    auto rawData = dynamic_cast<VolumeRAMPrecision<glm::u16>*>(
                       segmentationVolume->getEditableRepresentation<VolumeRAM>())
                       ->getDataTyped();

    /*
     * Look up which feature the arcId in arcMap belongs to and assign value. That should be it.
     */
    glm::u16 n{static_cast<glm::u16>(topKFeatures - 1)};
    const auto numberOfElements = glm::compMul(inputVolume->getDimensions());

    std::fill_n(rawData, numberOfElements, 0);

    std::transform(/*std::execution::par_unseq,*/ std::cbegin(arcMap_), std::cend(arcMap_), rawData,
                   [features](const auto voxelArcId) {
                       const auto numberOfFeatures = features.size();

                       for (size_t j{1}; j < numberOfFeatures; ++j) {
                           if (const auto it = std::find_if(std::cbegin(features[j].arcs),
                                                            std::cend(features[j].arcs),
                                                            [voxelArcId](const auto featureArcId) {
                                                                return voxelArcId == featureArcId;
                                                            });
                               it != features[j].arcs.end()) {
                               return static_cast<glm::u16>(j);
                           }
                       }

                       return glm::u16{0};
                   });

    // std::for_each(std::execution::par_unseq, std::begin(features), std::end(features),
    //              [&features, rawData, numberOfElements, this](const auto& feature) {
    //                  //
    //                  https://stackoverflow.com/questions/3752019/how-to-get-the-index-of-a-value-in-a-vector-using-for-each
    //                  const auto idx = &feature - &features[0];

    //                  for (size_t i{0}; i < numberOfElements; ++i) {
    //                      const auto it = std::find(std::cbegin(feature.arcs),
    //                      std::cend(feature.arcs),
    //                                          arcMap_[i]);

    //                      if (it != std::cend(feature.arcs)) {
    //                          rawData[i] = static_cast<glm::u16>(idx);
    //                      }
    //                  }
    //              });
    
    auto printHistogram = [rawData,
                           numberOfElements = glm::compMul(inputVolume->getDimensions())]() {
        std::map<glm::u16, size_t> histogram;
        for (size_t i{0}; i < numberOfElements; ++i) {
            histogram[rawData[i]]++;
        }

        for (auto pair : histogram) {
            LogInfoCustom("ContourTreeProcessor", fmt::format("{0}: {1}", pair.first, pair.second));
        }
    };

    // printHistogram();

    segmentationVolume->dataMap_.dataRange = segmentationVolume->dataMap_.valueRange =
        dvec2{0, features.size() - 1};


    LogInfo(fmt::format("# features:      {}", features.size()));
    LogInfo(fmt::format("Max element CPU: {}", *std::max_element(rawData, rawData + numberOfElements)));
    LogInfo(fmt::format("Max element GPU: {}",
                        VolumeReductionGL().reduce_v(segmentationVolume, ReductionOperator::Max)));
    LogInfo(fmt::format("Data range:      {}", segmentationVolume->dataMap_.dataRange.y));

    segmentationVolume->setBasis(inputVolume->getBasis());
    segmentationVolume->setOffset(inputVolume->getOffset());

    segmentationVolume->setInterpolation(InterpolationType::Nearest);

    segmentationOutport_.setData(segmentationVolume);
}

}  // namespace inviwo
