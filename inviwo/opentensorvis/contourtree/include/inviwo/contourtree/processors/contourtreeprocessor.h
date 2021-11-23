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

#pragma once

#include <inviwo/contourtree/contourtreemoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/templateproperty.h>
#include <ContourTreeData.h>
#include <SimplifyCT.h>
#include <constants.h>

#include "TopologicalFeatures.h"

namespace inviwo {

/** \docpage{org.inviwo.ContourTreeProcessor, Contour Tree}
 * ![](org.inviwo.ContourTreeProcessor.png?classIdentifier=org.inviwo.ContourTreeProcessor)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *   * __<Outport2>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */

/**
 * \brief VERY_BRIEFLY_DESCRIBE_THE_PROCESSOR
 * DESCRIBE_THE_PROCESSOR_FROM_A_DEVELOPER_PERSPECTIVE
 */
class IVW_MODULE_CONTOURTREE_API ContourTreeProcessor : public Processor {
public:
    ContourTreeProcessor();
    virtual ~ContourTreeProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    enum class FeatureType { Arc, PartitionedExtrema };

    enum class QueryCriterion { TopKFeatures, Threshold };

    enum class SimplificationMetod { Persistence, Hypervolume };

    VolumeInport volumeInport_;

    VolumeOutport segmentationOutport_;

    TemplateOptionProperty<contourtree::TreeType> treeType_;
    TemplateOptionProperty<FeatureType> featureType_;
    TemplateOptionProperty<QueryCriterion> queryCriterion_;
    TemplateOptionProperty<SimplificationMetod> simplificationMetod_;
    
    IntProperty topKFeatures_;
    FloatProperty threshold_;

    bool hasData_;
    std::vector<uint32_t> arcMap_;
    contourtree::TopologicalFeatures topologicalFeatures_;

    void computeTree();
};

}  // namespace inviwo
