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

#include <inviwo/contourtree/processors/contourtreequeryprocessor.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/contourtree/util/util.h>
#include <Persistence.h>
#include <Grid3D.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeQueryProcessor::processorInfo_{
    "org.inviwo.ContourTreeQueryProcessor",         // Class identifier
    "Contour Tree Query",                           // Display name
    "OpenTensorVis",                                // Category
    CodeState::Experimental,                        // Code state
    "topology, merge tree, join tree, split tree",  // Tags
};
const ProcessorInfo ContourTreeQueryProcessor::getProcessorInfo() const { return processorInfo_; }

ContourTreeQueryProcessor::ContourTreeQueryProcessor()
    : Processor()
    , volumeInport_("volumeInport")
    , contourTreeInport_("contourTreeInport")
    , contourTreeDataInport_("contourTreeDataInport")
    , contourTreeSimplificationInport_("contourTreeSimplificationInport")
    , contourTreeTopologicalFeatuesInport_("contourTreeTopologicalFeatuesInport")
    , volumeOutport_("volumeOutport")
    , queryMethod_("queryMethod", "Query method",
                   {{"option_methodTopoAngler", "TopoAngler", QueryMethod::TopoAngler},
                    {"option_methodCutoff", "Cutoff", QueryMethod::Cutoff},
                    {"option_methodLeaves", "Leaves", QueryMethod::Leaves}},
                   0)
    , methodTopoAngler_("methodTopoAngler", "TopoAngler")

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
    , persistence_("persistence", "Persistence") {
    addPorts(volumeInport_, contourTreeInport_, contourTreeDataInport_,
             contourTreeSimplificationInport_, contourTreeTopologicalFeatuesInport_,
             volumeOutport_);

    addProperties(queryMethod_, methodTopoAngler_, methodCutoff_, methodNLeaves_);

    contourTreeSimplificationInport_.onChange([this]() {
        const auto contourTreeSimplification = contourTreeSimplificationInport_.getData();

        const auto simplificationType = contourTreeSimplification->simFn->simType_;

        switch (simplificationType) {
            case contourtree::SimFunction::SimType::Persistence: {
                NetworkLock l;

                const auto persistenceSimplification =
                    std::dynamic_pointer_cast<contourtree::Persistence>(
                        contourTreeSimplification->simFn);

                const auto min = persistenceSimplification->getMinPersistence();
                const auto max = persistenceSimplification->getMaxPersistence();

                if (persistence_.get() < min && persistence_.get() > max) {
                    persistence_.set(min);
                }

                persistence_.setMinValue(min);
                persistence_.setMaxValue(max);
            }

            break;
            case contourtree::SimFunction::SimType::HyperVolume:
                break;
        }
    });

    volumeInport_.onChange([this]() {
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

    methodTopoAngler_.addProperties(queryCriterion_, topKFeatures_, threshold_);

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
    methodNLeaves_.addProperties(nLeaves_, persistence_);
}

void ContourTreeQueryProcessor::process() {
    switch (queryMethod_.get()) {
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

void ContourTreeQueryProcessor::queryTopoAngler() {
    if (!util::checkPorts(contourTreeInport_, contourTreeTopologicalFeatuesInport_)) return;

    const auto contourTree = contourTreeInport_.getData();
    const auto topologicalFeatures = contourTreeTopologicalFeatuesInport_.getData();

    std::vector<contourtree::Feature> features;

    const auto topKFeatures =
        queryCriterion_.get() == QueryCriterion::TopKFeatures ? topKFeatures_.get() : -1;

    if (!topologicalFeatures->isPartitioned) {
        features = topologicalFeatures->getArcFeatures(topKFeatures, threshold_.get());

        /*
         * Look up which feature the arcId in arcMap belongs to and assign value. That should be it.
         */
        const auto numberOfElements = contourTree->noVertices;

        auto rawData = new uint16_t[numberOfElements];
        std::fill_n(rawData, numberOfElements, 0);

        for (size_t i{0}; i < features.size(); ++i) {
            for (const auto arcId : features[i].arcs) {
                for (size_t j{0}; j < numberOfElements; ++j) {
                    if (contourTree->arcMap[j] == arcId) {
                        rawData[j] = static_cast<uint16_t>(i + 1);
                    }
                }
            }
        }

        generateSegmentationVolume(rawData, features.size());

    } else {
        features =
            topologicalFeatures->getPartitionedExtremaFeatures(topKFeatures, threshold_.get());

        /*
         * Look up which feature the arcId in arcMap belongs to and assign value. That should be it.
         */
        const auto numberOfElements = contourTree->noVertices;

        auto rawData = new uint16_t[numberOfElements];
        std::fill_n(rawData, numberOfElements, 0);

        for (size_t i{1}; i < features.size(); ++i) {
            for (const auto arcId : features[i].arcs) {
                for (size_t j{0}; j < numberOfElements; ++j) {
                    if (contourTree->arcMap[j] == arcId) {
                        rawData[j] = static_cast<uint16_t>(i);
                    }
                }
            }
        }

        generateSegmentationVolume(rawData, features.size() - 1);
    }
}

void ContourTreeQueryProcessor::queryCutoff() { auto intersectingArcs = getIntersectingArcs(); }

void ContourTreeQueryProcessor::queryNLeaves() {
    if (!util::checkPorts(volumeInport_,contourTreeInport_, contourTreeDataInport_,
                          contourTreeSimplificationInport_))
        return;

    const auto contourTree = contourTreeInport_.getData();
    const auto contourTreeData = contourTreeDataInport_.getData();
    const auto contourTreeSimplification = contourTreeSimplificationInport_.getData();

    const auto topologicalFeatures = contourTreeTopologicalFeatuesInport_.getData();

    const auto features =
        volumeInport_.getData()
            ->getRepresentation<VolumeRAM>()
            ->dispatch<std::vector<contourtree::Feature>,
                       dispatching::filter::Scalars>([&, this](auto vrprecision) {
                using ValueType = util::PrecisionValueType<decltype(vrprecision)>;

                const auto grid = std::dynamic_pointer_cast<const contourtree::Grid3D<ValueType>>(
                    contourTree->data);

                std::vector<contourtree::Feature> f;

                if (contourTree->treeType_ == contourtree::TreeType::JoinTree) {
                    f = topologicalFeatures
                                   ->getNExtremalArcFeatures<contourtree::TreeType::JoinTree, ValueType>(
                                       nLeaves_, persistence_.get(), grid->fnVals);
                }

                if (contourTree->treeType_ == contourtree::TreeType::SplitTree) {
                    f = topologicalFeatures
                            ->getNExtremalArcFeatures<contourtree::TreeType::SplitTree, ValueType>(
                                       nLeaves_, persistence_.get(), grid->fnVals);
                }

                if (f.size() < nLeaves_.get()) {
                    LogWarn(fmt::format(
                        "With persistence simplification level at {} the tree yielded {} features "
                        "instead of the requested {}.",
                        persistence_.get(), f.size(), nLeaves_.get()));
                }

                return f;
            });

    const auto numberOfElements = contourTree->noVertices;

    auto rawData = new uint16_t[numberOfElements];
    std::fill_n(rawData, numberOfElements, 0);

    for (size_t i{0}; i < features.size(); ++i) {
        for (const auto arcId : features[i].arcs) {
            for (size_t j{0}; j < numberOfElements; ++j) {
                if (contourTree->arcMap[j] == arcId) {
                    rawData[j] = static_cast<uint16_t>(i + 1);
                }
            }
        }
    }

    generateSegmentationVolume(rawData, features.size());
}

std::vector<std::pair<uint32_t, uint32_t>> ContourTreeQueryProcessor::getIntersectingArcs() {
    if (!util::checkPorts(contourTreeDataInport_, volumeInport_))
        return std::vector<std::pair<uint32_t, uint32_t>>{};

    const auto cutoff = cutoff_.get();

    const auto inputVolume = volumeInport_.getData();

    return inputVolume->getRepresentation<VolumeRAM>()
        ->dispatch<std::vector<std::pair<uint32_t, uint32_t>>, dispatching::filter::Scalars>(
            [&, this](auto vrprecision) {
                using ValueType = util::PrecisionValueType<decltype(vrprecision)>;
                auto volumeData = vrprecision->getDataTyped();

                const auto contourTreeData = contourTreeDataInport_.getData();

                const auto& arcs = contourTreeData->arcs;

                std::vector<std::pair<uint32_t, uint32_t>> intersectingArcs;

                auto inRange = [&](const ValueType a, const ValueType b) {
                    return cutoff >= static_cast<float>(a) && cutoff <= static_cast<float>(b);
                };

                size_t numberOfIntersectingArcs = 0;

                for (const auto [from, to, id] : arcs) {
                    const auto value1 = volumeData[contourTreeData->nodeVerts[from]];
                    const auto value2 = volumeData[contourTreeData->nodeVerts[to]];

                    if (inRange(value1, value2)) {
                        numberOfIntersectingArcs++;
                    }
                }

                LogInfoCustom("ContourTreeComputationProcessor",
                              fmt::format("{} intersecting arcs at cutoff {}.",
                                          numberOfIntersectingArcs, cutoff));

                return intersectingArcs;
            });
}

std::vector<std::pair<uint32_t, uint32_t>>
ContourTreeQueryProcessor::getNLeavesAndCorrespondingArcs() {
    if (!util::checkPorts(contourTreeDataInport_))
        return std::vector<std::pair<uint32_t, uint32_t>>{};

    const auto contourTreeData = contourTreeDataInport_.getData();

    const auto n = nLeaves_.get();

    std::vector<std::pair<uint32_t, uint32_t>> list;

    for (uint32_t i{0}; i < n; ++i) {
        for (uint32_t j{0}; j < contourTreeData->arcs.size(); ++j) {
            if (const auto [from, to, id] = contourTreeData->arcs[j]; from == i) {
                list.emplace_back(i, j);
                break;
            }
        }
    }

    LogInfo(fmt::format("Extracted {} node/arc pairs for {} features.", list.size(), n));

    return list;
}

void ContourTreeQueryProcessor::generateSegmentationVolume(uint16_t* rawData, const glm::u8 n) {
    const auto inputVolume = volumeInport_.getData();
    const auto contourTree = contourTreeInport_.getData();

    const auto segmentationVolumeRamPrecision =
        std::make_shared<VolumeRAMPrecision<uint16_t>>(rawData, inputVolume->getDimensions());

    auto segmentationVolume = std::make_shared<Volume>(segmentationVolumeRamPrecision);

    segmentationVolume->dataMap_.dataRange = segmentationVolume->dataMap_.valueRange = dvec2{0, n};
    segmentationVolume->setBasis(inputVolume->getBasis());
    segmentationVolume->setOffset(inputVolume->getOffset());

    segmentationVolume->setInterpolation(InterpolationType::Nearest);

    volumeOutport_.setData(segmentationVolume);
}

}  // namespace inviwo
