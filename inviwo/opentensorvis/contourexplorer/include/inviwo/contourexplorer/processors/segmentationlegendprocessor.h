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

#include <inviwo/contourexplorer/contourexplorermoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/volumeport.h>
#include <modules/brushingandlinking/ports/brushingandlinkingports.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/nanovgutils/nanovgcontext.h>
#include <modules/fontrendering/properties/fontproperty.h>
#include <inviwo/contourtree/ports/extremalpointsport.h>

namespace inviwo {

/** \docpage{org.inviwo.SegmentationLegendProcessor, Segmentation Legend Processor}
 * ![](org.inviwo.SegmentationLegendProcessor.png?classIdentifier=org.inviwo.SegmentationLegendProcessor)
 */
class IVW_MODULE_CONTOUREXPLORER_API SegmentationLegendProcessor : public Processor {
public:
    SegmentationLegendProcessor();
    virtual ~SegmentationLegendProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    ExtremalPointsInport segmentMinimaInport_;
    BrushingAndLinkingInport brushingAndLinkingInport_;

    ImageInport imageInport_;
    ImageOutport imageOutport_;

    FloatProperty height_;
    FloatProperty marginBottom_;
    FloatProperty marginLeft_;
    FontProperty fontProperties_;
    FloatVec4Property fontColor_;

    NanoVGContext& nvgContext_;
};

}  // namespace inviwo
