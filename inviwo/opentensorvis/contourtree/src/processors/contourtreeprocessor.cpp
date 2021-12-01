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
#include <inviwo/core/util/zip.h>
#include <inviwo/core/util/indexmapper.h>
#include <modules/base/algorithm/meshutils.h>
#include "inviwo/core/datastructures/geometry/typedmesh.h"

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
    , meshOutport_("meshOutport")
    , queryMethod_("queryMethod", "Query method",
                   {{"methodHarish", "TopoAngler", QueryMethod::TopoAngler},
                    {"methodCutoff_", "Cutoff", QueryMethod::Cutoff},
                    {"methodLeaves", "Leaves", QueryMethod::Leaves}},
                   0)
    , methodGeneral_("methodGeneral", "General")
    , treeType_("treeType", "Tree type",
                {{"join", "Join", contourtree::TreeType::JoinTree},
                 {"split", "Split", contourtree::TreeType::SplitTree}},
                0)
    , simplificationMetod_("simplificationMetod", "Simplification method",
                           {{"persistence", "Persistence", SimplificationMetod::Persistence},
                            {"hypervolume", "Hypervolume", SimplificationMetod::Hypervolume}},
                           1)
    , methodTopoAngler_("methodTopoAngler", "TopoAngler")
    , featureType_("featureType", "Feature type",
                   {{"arc", "Arc", FeatureType::Arc},
                    {"partitionedExtrema", "Partitioned extrema", FeatureType::PartitionedExtrema}},
                   1)
    , queryCriterion_("queryCriterion", "Query criterion",
                      {{"topk", "Top k features", QueryCriterion::TopKFeatures},
                       {"threshold", "Threshold", QueryCriterion::Threshold}},
                      0)
    , topKFeatures_("topKFeatures", "Top k features", 1, 1, 12, 1)
    , threshold_("threshold", "Threshold", 0.0f, 0.0f, 1.0f, 0.0001f)
    , methodCutoff_("methodCutoff", "Cutoff")
    , cutoff_("cutoff", "Function value")
    , methodNLeaves_("methodNLeaves", "N Leaves")
    , nLeaves_("nLeaves", "Number of leaves", 1, 1, 12, 1)
    , sphereOptions_("sphereOptions", "Sphere options")
    , radius_("radius", "Radius", 0.1f, 0.1f, 10.f, 0.1f)
    , color_("color", "Color", vec4(1), vec4(0), vec4(1), vec4(0.00001f),
             InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , hasData_(false) {
    auto updateTree = [this]() { computeTree(); };
    auto updateMesh = [this]() { generateMesh(); };

    volumeInport_.onChange([this]() {
        computeTree();

        NetworkLock l;

        auto inputVolume = volumeInport_.getData();
        const auto min = static_cast<float>(inputVolume->dataMap_.dataRange.x);
        const auto max = static_cast<float>(inputVolume->dataMap_.dataRange.y);

        if (cutoff_.get() < min && cutoff_.get() > max) {
            cutoff_.set(min);
        }

        cutoff_.setMinValue(min);
        cutoff_.setMaxValue(max);
    });
    addPorts(volumeInport_, segmentationOutport_, meshOutport_);

    /**
     * General method setup
     */
    treeType_.onChange(updateTree);
    simplificationMetod_.onChange(updateTree);
    methodGeneral_.addProperties(treeType_, simplificationMetod_);

    /**
     * Setup for TopoAngler's method
     */
    methodTopoAngler_.visibilityDependsOn(queryMethod_, [](TemplateOptionProperty<QueryMethod>& p) {
        return p.get() == QueryMethod::TopoAngler;
    });
    topKFeatures_.visibilityDependsOn(queryCriterion_,
                                      [](TemplateOptionProperty<QueryCriterion>& p) {
                                          return p.get() == QueryCriterion::TopKFeatures;
                                      });
    threshold_.visibilityDependsOn(queryCriterion_, [](TemplateOptionProperty<QueryCriterion>& p) {
        return p.get() == QueryCriterion::Threshold;
    });
    featureType_.onChange(updateTree);
    methodTopoAngler_.addProperties(featureType_, queryCriterion_, topKFeatures_, threshold_);

    /**
     * Setup for Cutoff method
     */
    methodCutoff_.visibilityDependsOn(queryMethod_, [](TemplateOptionProperty<QueryMethod>& p) {
        return p.get() == QueryMethod::Cutoff;
    });
    methodCutoff_.addProperties(cutoff_);

    /**
     * Setup for nLeaves method
     */
    methodNLeaves_.visibilityDependsOn(queryMethod_, [](TemplateOptionProperty<QueryMethod>& p) {
        return p.get() == QueryMethod::Leaves;
    });
    methodNLeaves_.addProperties(nLeaves_);

    /**
     * Mesh options
     */
    radius_.onChange(updateMesh);
    color_.onChange(updateMesh);
    sphereOptions_.addProperties(radius_, color_);

    addProperties(queryMethod_, methodGeneral_, methodTopoAngler_, methodCutoff_, methodNLeaves_,
                  sphereOptions_);
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

    inputVolume->getRepresentation<VolumeRAM>()->dispatch<void, dispatching::filter::Scalars>(
        [&, this](auto vrprecision) {
            using ValueType = util::PrecisionValueType<decltype(vrprecision)>;

            const auto dimensions = inputVolume->getDimensions();

            contourtree::Grid3D<ValueType> grid(static_cast<int>(dimensions.x),
                                                static_cast<int>(dimensions.y),
                                                static_cast<int>(dimensions.z));

            grid.fnVals.resize(glm::compMul(dimensions));
            auto gridData = grid.fnVals.data();

            auto volumeData = vrprecision->getDataTyped();

            std::memcpy(gridData, volumeData, glm::compMul(dimensions) * sizeof(ValueType));

            const auto treeType = treeType_.get();

            mergeTree_ = contourtree::MergeTree();

            mergeTree_.computeTree(&grid, treeType);

            mergeTree_.generateArrays(treeType);

            contourTreeData_ = contourtree::ContourTreeData(mergeTree_);
        });

    //generateMesh();

    hasData_ = true;
    isSimplified_ = false;
}

void ContourTreeProcessor::simplifyTree() {
    if (!hasData_) return;

    simplifyCt_ = contourtree::SimplifyCT();

    simplifyCt_.setInput(&contourTreeData_);

    contourtree::SimFunction* simFn;

    if (simplificationMetod_.get() == SimplificationMetod::Persistence) {
        simFn = new contourtree::Persistence(contourTreeData_);
    } else {
        simFn = new contourtree::HyperVolume(contourTreeData_, mergeTree_.arcMap);
    }

    delete simplifyCt_.simFn;

    simplifyCt_.simplify(simFn);
    simplifyCt_.computeWeights();
}

size_t ContourTreeProcessor::findRoot() {
    if (!hasData_) return 0;

    auto& nodes = contourTreeData_.nodes;

    contourtree::Node root;

    size_t i{0};
    for (; i < nodes.size(); i++) {
        if (nodes[i].next.empty()) {
            break;
        }
    }

    return i;
}

std::vector<std::pair<uint32_t, uint32_t>> ContourTreeProcessor::getIntersectingArcs() {
    const auto cutoff = cutoff_.get();

    if (!hasData_) return std::vector<std::pair<uint32_t, uint32_t>>{};

    const auto inputVolume = volumeInport_.getData();

    return inputVolume->getRepresentation<VolumeRAM>()
        ->dispatch<std::vector<std::pair<uint32_t, uint32_t>>, dispatching::filter::Scalars>(
            [&, this](auto vrprecision) {
                using ValueType = util::PrecisionValueType<decltype(vrprecision)>;
                auto volumeData = vrprecision->getDataTyped();

                const auto& contourTreeData = topologicalFeatures_.ctdata;

                const auto& arcs = contourTreeData.arcs;

                std::vector<std::pair<uint32_t, uint32_t>> intersectingArcs;

                auto inRange = [&](const ValueType a, const ValueType b) {
                    return cutoff >= static_cast<float>(a) && cutoff <= static_cast<float>(b);
                };

                size_t numberOfIntersectingArcs = 0;

                for (const auto [from, to, id] : arcs) {
                    const auto value1 = volumeData[contourTreeData.nodeVerts[from]];
                    const auto value2 = volumeData[contourTreeData.nodeVerts[to]];

                    if (inRange(value1, value2)) {
                        numberOfIntersectingArcs++;
                    }
                }

                LogInfoCustom("ContourTreeProcessor",
                              fmt::format("{} intersecting arcs at cutoff {}.",
                                          numberOfIntersectingArcs, cutoff));

                return intersectingArcs;
            });
}

std::vector<std::pair<uint32_t, uint32_t>> ContourTreeProcessor::getNLeavesAndCorrespondingArcs() {
    if (!hasData_) computeTree();
    if (!isSimplified_) simplifyTree();

    const auto n = nLeaves_.get();

    std::vector<std::pair<uint32_t, uint32_t>> list;

    if (!hasData_) return list;

    for (uint32_t i{0}; i < n; ++i) {
        for (uint32_t j{0}; j < topologicalFeatures_.ctdata.arcs.size(); ++j) {
            if (const auto [from, to, id] = topologicalFeatures_.ctdata.arcs[j]; from == i) {
                list.emplace_back(i, j);
                break;
            }
        }
    }

    LogInfo(fmt::format("Extracted {} node/arc pairs for {} features.", list.size(), n));

    return list;
}

void ContourTreeProcessor::generateMesh() {
    if (!volumeInport_.hasData() || !volumeInport_.getData()) return;

    auto inputVolume = volumeInport_.getData();

    std::vector<vec3> positions;

    util::IndexMapper3D indexMapper(inputVolume->getDimensions());

    const auto& nodeVertices = topologicalFeatures_.ctdata.nodeVerts;

    for (int i{0}; i < topKFeatures_.get(); ++i) {
        auto index = nodeVertices[i];
        auto position = vec3(indexMapper(index));
        positions.push_back(position);
    }

    auto outputMesh = std::make_shared<BasicMesh>();

    for (auto& position : positions) {
        position = inputVolume->getBasis() * (vec3(position) / vec3(inputVolume->getDimensions()));

        auto mesh = meshutil::sphere(position, radius_.get(), color_.get());
        mesh->setModelMatrix(inputVolume->getModelMatrix());
        mesh->setWorldMatrix(inputVolume->getWorldMatrix());
        outputMesh->Mesh::append(*mesh);
    }

    meshOutport_.setData(outputMesh);
}

void ContourTreeProcessor::query(const QueryMethod method) {
    switch (method) {
        case QueryMethod::TopoAngler:
            queryTopoAngler();
            break;
        case QueryMethod::Cutoff:
            queryCutoff();
            break;
        case QueryMethod::Leaves:
            queryNLeaves();
            break;
    }
}

void ContourTreeProcessor::queryTopoAngler() {
    if (!isSimplified_) simplifyTree();

    const auto partition = featureType_.get() == FeatureType::PartitionedExtrema;

    topologicalFeatures_.loadDataFromArrays(contourTreeData_, simplifyCt_.order,
                                            simplifyCt_.weights, partition);

    std::vector<contourtree::Feature> features;

    const int topKFeatures =
        queryCriterion_.get() == QueryCriterion::TopKFeatures ? topKFeatures_.get() : -1;

    if (featureType_.get() == FeatureType::Arc) {
        features = topologicalFeatures_.getArcFeatures(topKFeatures, threshold_.get());
    } else {
        features =
            topologicalFeatures_.getPartitionedExtremaFeatures(topKFeatures, threshold_.get());
    }

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

    generateSegmentationVolume(rawData, features.size() - 1);
}

void ContourTreeProcessor::queryCutoff() { auto intersectingArcs = getIntersectingArcs(); }

void ContourTreeProcessor::queryNLeaves() {
    auto nLeavesAndCorrespondingArcs = getNLeavesAndCorrespondingArcs();

    auto numberOfElements = glm::compMul(volumeInport_.getData()->getDimensions());

    auto rawData = new uint16_t[numberOfElements];
    std::fill_n(rawData, numberOfElements, nLeavesAndCorrespondingArcs.size() + 1);

    for (size_t i{}; i < nLeavesAndCorrespondingArcs.size(); ++i) {
        const auto& arc = contourTreeData_.arcs[nLeavesAndCorrespondingArcs[i].second];
        const auto arcId = arc.id;

        for (size_t j{0}; j < numberOfElements; ++j) {
            if (arcMap_[j] == arcId) {
                rawData[j] = static_cast<uint16_t>(i);
            }
        }
    }

    generateSegmentationVolume(rawData, nLeavesAndCorrespondingArcs.size() - 1);
}

void ContourTreeProcessor::generateSegmentationVolume(uint16_t* rawData, const glm::u8 n) {
    const auto inputVolume = volumeInport_.getData();

    const auto segmentationVolumeRamPrecision =
        std::make_shared<VolumeRAMPrecision<uint16_t>>(rawData, inputVolume->getDimensions());

    auto segmentationVolume = std::make_shared<Volume>(segmentationVolumeRamPrecision);

    segmentationVolume->dataMap_.dataRange = segmentationVolume->dataMap_.valueRange = dvec2{0, n};
    segmentationVolume->setBasis(inputVolume->getBasis());
    segmentationVolume->setOffset(inputVolume->getOffset());

    segmentationVolume->setInterpolation(InterpolationType::Nearest);

    segmentationOutport_.setData(segmentationVolume);
}

void ContourTreeProcessor::process() {
    if (!hasData_) computeTree();

    query(queryMethod_.get());
}

}  // namespace inviwo

/* auto inRange = [](const float threshold, const ValueType a, const ValueType b) {
                        return threshold >= static_cast<float>(a) &&
                               threshold <= static_cast<float>(b);
                    };

                    auto volumeData = vrprecision->getDataTyped();
                    const auto& nodeVertices = topologicalFeatures_.ctdata.nodeVerts;
                    auto nodes = topologicalFeatures_.ctdata.nodes;

                    auto parentIndex = findRoot();
                    auto& parentNode = nodes[parentIndex];

                    std::vector<std::pair<uint32_t, uint32_t>> intersectingArcs;

                    std::vector<std::pair<uint32_t, uint32_t>> nodesToInvestigate;

                    for (auto id : parentNode.prev) {
                        nodesToInvestigate.emplace_back(parentIndex, id);
                    }

                    while (!nodesToInvestigate.empty()) {

                        auto [parent, kid] = nodesToInvestigate.back();
                        nodesToInvestigate.pop_back();

                        const ValueType nodeValue = volumeData[nodeVertices[kid]];
                        const ValueType parentValue = volumeData[nodeVertices[parent]];

                        if (inRange(threshold, nodeValue,parentValue)) {
                            intersectingArcs.emplace_back(parent, kid);
                        } else {
                            for (auto id : nodes[kid].prev)
                                nodesToInvestigate.emplace_back(kid, id);
                        }
                    }
*/