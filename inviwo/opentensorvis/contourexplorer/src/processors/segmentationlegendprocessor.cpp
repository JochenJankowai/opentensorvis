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

#include <inviwo/contourexplorer/processors/segmentationlegendprocessor.h>
#include <inviwo/contourtree/util/util.h>
#include <inviwo/core/util/colorbrewer.h>
#include <inviwo/nanovgutils/nanovgutilsmodule.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/texture/textureutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SegmentationLegendProcessor::processorInfo_{
    "org.inviwo.SegmentationLegendProcessor",  // Class identifier
    "Segmentation Legend",                     // Display name
    "OpenTensorVis",                           // Category
    CodeState::Experimental,                   // Code state
    Tags::None,                                // Tags
};
const ProcessorInfo SegmentationLegendProcessor::getProcessorInfo() const { return processorInfo_; }

SegmentationLegendProcessor::SegmentationLegendProcessor()
    : Processor()
    , segmentMinimaInport_("segmentMinimaInport")
    , brushingAndLinkingInport_("brushingAndLinkingInport")
    , imageOutport_("imageOutport")
    , height_("height", "Height", 20, 10, 200, 2)
    , marginBottom_("marginBottom", "Margin bottom", 20, 0, 200, 2)
    , marginLeft_("marginLeft", "Margin left", 20, 0, 200, 2)
    , fontProperties_("fontProperties", "Font properties")
    , fontColor_("","")
    , nvgContext_{
          InviwoApplication::getPtr()->getModuleByType<NanoVGUtilsModule>()->getNanoVGContext()} {

    addPorts(segmentMinimaInport_, brushingAndLinkingInport_, imageOutport_);

    fontColor_.setSemantics(PropertySemantics::Color);
    fontColor_.setCurrentStateAsDefault();
    fontProperties_.addProperties(fontColor_);
    addProperties(height_, marginBottom_, marginLeft_,fontProperties_);

    auto loadFontIfNeeded = [this]() {
        auto fontFace = fontProperties_.fontFace_.getSelectedValue();
        auto fontName = fontProperties_.fontFace_.getSelectedIdentifier();

        if (nvgContext_.findFont(fontFace) == -1) {
            if (nvgContext_.createFont(fontName, fontFace) == -1) {
                LogError(fmt::format("Could not create font: {}", fontFace));
            }
        }

        invalidate(InvalidationLevel::InvalidOutput);
    };

    loadFontIfNeeded();

    fontProperties_.fontFace_.onChange(loadFontIfNeeded);
}

void SegmentationLegendProcessor::process() {
    if (!util::checkPorts(segmentMinimaInport_)) return;

    const auto& segmentMinima = *segmentMinimaInport_.getData();

    const auto max =
        std::max_element(std::begin(segmentMinima), std::end(segmentMinima));
        
    const auto numberOfSegments = static_cast<float>(max->second) + 1.0f;
    const auto iNumberOfSegments = static_cast<size_t>(max->second) + 1;

    // Get color map
    const auto& colorMap = colorbrewer::getColormap(colorbrewer::Family::Set3,
                                                    static_cast<glm::uint8>(numberOfSegments));

    auto drawValues = [this](const vec2& at, const vec2& values) {
        auto text = std::string{fmt::format("[{:03.4f}, {:03.4f}]", values.x, values.y)};
        auto bounds = nvgContext_.textBounds(at, text);

        nvgContext_.beginPath();
        nvgContext_.fontSize(static_cast<float>(fontProperties_.fontSize_.get()));
        nvgContext_.fontFace(fontProperties_.fontFace_.getSelectedIdentifier());
        nvgContext_.textAlign(NanoVGContext::Alignment::Center_Bottom);
        nvgContext_.fillColor(fontColor_.get());
        nvgContext_.textBox(at, bounds[2] - bounds[0], text);
        nvgContext_.closePath();
    };

    auto drawRectangle = [&, this](const vec2& position, const size2_t& dimensions,
                                   const dvec4& color) {
        nvgContext_.beginPath();
        nvgContext_.rect(position, vec2(dimensions));
        nvgContext_.fillColor(color);
        nvgContext_.fill();
        nvgContext_.closePath();
    };

    // Generate positions
    const auto segments = 2.0f + (numberOfSegments - 1.0f) + (numberOfSegments * 2.0f);

    auto dimensions = vec2(imageOutport_.getDimensions());

    utilgl::activateAndClearTarget(imageOutport_);

    nvgContext_.activate(dimensions);

    dimensions.x -= marginLeft_.get();

    const auto gapWidth = dimensions.x / segments;

    const auto rectangleWidth = gapWidth * 2;

    std::vector<vec2> positions{};

    for (size_t i{0}; i < iNumberOfSegments; ++i) {
        positions.emplace_back((gapWidth * static_cast<float>(i + 1)) +
                                   (rectangleWidth * static_cast<float>(i)) + marginLeft_.get(),
                               dimensions.y - marginBottom_.get() - height_.get());
    }
    
    for (size_t i{0}; i < iNumberOfSegments; ++i) {
        drawRectangle(positions[i], size2_t{rectangleWidth, height_.get()}, colorMap[i]);
        drawValues(positions[i] + vec2(0, height_.get()/2.0f), positions[i]);
    }

    nvgContext_.deactivate();

    utilgl::deactivateCurrentTarget();
}

}  // namespace inviwo
