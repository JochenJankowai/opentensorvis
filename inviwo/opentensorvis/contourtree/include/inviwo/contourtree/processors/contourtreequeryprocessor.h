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
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/contourtree/ports/contourtreeport.h>
#include <inviwo/contourtree/ports/contourtreedataport.h>
#include <inviwo/contourtree/ports/contourtreesimplificationport.h>
#include <inviwo/contourtree/ports/contourtreetopologicalfeaturesport.h>

namespace inviwo {

/** \docpage{org.inviwo.ContourTreeQueryProcessor, Contour Tree Query Processor}
 * ![](org.inviwo.ContourTreeQueryProcessor.png?classIdentifier=org.inviwo.ContourTreeQueryProcessor)
 */
class IVW_MODULE_CONTOURTREE_API ContourTreeQueryProcessor : public Processor {
public:
    ContourTreeQueryProcessor();
    virtual ~ContourTreeQueryProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    enum class QueryMethod { TopoAngler, Cutoff, Leaves };
    enum class QueryCriterion { TopKFeatures, Threshold };

    VolumeInport volumeInport_;
    ContourTreeInport contourTreeInport_;
    ContourTreeDataInport contourTreeDataInport_;
    ContourTreeSimplificationInport contourTreeSimplificationInport_;
    ContourTreeTopologicalFeaturesInport contourTreeTopologicalFeatuesInport_;

    VolumeOutport volumeOutport_;
    
    TemplateOptionProperty<QueryMethod> queryMethod_;

    /**
     * Properties for the original implementation by TopoAngler.
     */
    CompositeProperty methodTopoAngler_;

    TemplateOptionProperty<QueryCriterion> queryCriterion_;
    IntProperty topKFeatures_;
    FloatProperty threshold_;

    /**
     * Properties to steer method for extracting subtrees below a certain threshold.
     */
    CompositeProperty methodCutoff_;
    FloatProperty cutoff_;

    /**
     * Properties to steed method for extracting N leaves and their corresponding arcs.
     */
    CompositeProperty methodNLeaves_;
    IntProperty nLeaves_;
    FloatProperty simplificationThreshold_;
    

    void query(QueryMethod method);

    void queryTopoAngler();
    void queryCutoff();
    void queryNLeaves();
    
    void generateSegmentationVolume(uint16_t* rawData, glm::u8 n);
};

}  // namespace inviwo
