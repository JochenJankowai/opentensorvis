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
#include <inviwo/contourexplorer/util/segmentationcolorhelper.h>

#include "inviwo/core/interaction/events/mouseevent.h"

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

void SegmentationLegendProcessor::handleSelection() {
    auto dimensions = vec2(imageOutport_.getDimensions());
    const auto extents = vec2{rectangleWidth_, height_.get()};
    auto selection = brushingAndLinkingInport_.getSelectedIndices();
    auto flipY = [&](const vec2 pos) { return vec2{pos.x, dimensions.y - pos.y}; };

    /*
     * Unfortunately, picking has never been merged into nanovg. Therefore, we need to do it
     * ourselves. It'll be inaccurate since the rounded corners won't be respected but it'll do.
     */
    for (size_t i{0}; i < positions_.size();i++) {
        const auto hitPos = flipY(positions_[i]);
        if (isMouseDown_ && mousePos_.x >= hitPos.x && mousePos_.x <= hitPos.x + extents.x &&
            mousePos_.y <= hitPos.y && mousePos_.y >= hitPos.y - extents.y) {
            selection.flip(i);
            brushingAndLinkingInport_.select(selection);
            break;
        }
    }
}

SegmentationLegendProcessor::SegmentationLegendProcessor()
    : Processor()
    , segmentMinimaInport_("segmentMinimaInport")
    , brushingAndLinkingInport_("brushingAndLinkingInport")
    , imageInport_("imageInport")
    , imageOutport_("imageOutport")
    , styling_("styling", "Styling")
    , height_("height", "Height", 20, 10, 200, 2)
    , marginBottom_("marginBottom", "Margin bottom", 20, 0, 200, 2)
    , marginLeft_("marginLeft", "Margin left", 20, 0, 200, 2)
    , cornerRadius_("", "", 10, 0, 30, 2)
    , luminanceMultiplier_("luminanceMultiplier", "Luminance multiplier", 0.2f, 0.0f, 1.0f, 0.01f)
    , strokeWitdth_("strokeWitdth", "Stroke witdth", 1.0f, 1.0f, 10.0f, 0.1f)
    , fontProperties_("fontProperties", "Font properties")
    , fontColor_("fontColor", "Font color", dvec4{1}, dvec4{0, 0, 0, 1}, vec4{1}, dvec4{0.0001},
                 InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , nvgContext_{InviwoApplication::getPtr()
                      ->getModuleByType<NanoVGUtilsModule>()
                      ->getNanoVGContext()}
    , eventPropertySelection_(
          "eventPropertyClusterSelection", "Cluster selection",
          [this](Event* e) {
              if (const auto ev = e->getAs<MouseEvent>()) {
                  isMouseDown_ = true;
                  mousePos_ = ev->pos();
                  handleSelection();
              }
          },
          MouseButton::Left, MouseState::Press)
    , eventPropertyHighlighting_(
          "eventPropertyHighlighting", "Highlighting",
          [this](Event* e) {
              if (const auto ev = e->getAs<MouseEvent>()) {
                  isMouseDown_ = false;
                  mousePos_ = ev->pos();
                  invalidate(InvalidationLevel::InvalidOutput);
              }
          },
          MouseButton::None, MouseState::Move)
    , mousePos_(0.0f)
    , isMouseDown_(false) {

    imageInport_.setOptional(true);

    addPorts(imageInport_, segmentMinimaInport_, brushingAndLinkingInport_, imageOutport_);

    fontProperties_.addProperties(fontColor_);
    fontProperties_.fontSize_.setSemantics(PropertySemantics::Default);
    fontProperties_.setAllPropertiesCurrentStateAsDefault();
    fontProperties_.setCurrentStateAsDefault();

    eventPropertySelection_.setReadOnly(true);
    eventPropertySelection_.setVisible(false);
    eventPropertyHighlighting_.setReadOnly(true);
    eventPropertyHighlighting_.setVisible(false);

    styling_.addProperties(height_, marginBottom_, marginLeft_, cornerRadius_, luminanceMultiplier_,
                           strokeWitdth_);

    addProperties(styling_, fontProperties_, eventPropertySelection_, eventPropertyHighlighting_);

    auto loadFontIfNeeded = [this]() {
        const auto fontFace = fontProperties_.fontFace_.getSelectedValue();
        const auto& fontName = fontProperties_.fontFace_.getSelectedIdentifier();

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

    auto segmentMinima = *segmentMinimaInport_.getData();

    const auto iNumberOfSegments = segmentMinima.size();

    const auto numberOfSegments = static_cast<float>(iNumberOfSegments);

    auto selection = brushingAndLinkingInport_.getSelectedIndices();

    // Get color map
    auto colorMap = SegmentationColorHelper::getColorMapForNSegments(iNumberOfSegments);

    // Generate positions
    const auto segments = 2.0f + (numberOfSegments - 1.0f) + (numberOfSegments * 2.0f);

    auto dimensions = vec2(imageOutport_.getDimensions());

    if (imageInport_.isReady()) {
        utilgl::activateTargetAndCopySource(imageOutport_, imageInport_, ImageType::ColorDepth);
    } else {
        utilgl::activateAndClearTarget(imageOutport_, ImageType::ColorDepth);
    }

    nvgContext_.activate(dimensions);

    dimensions.x -= marginLeft_.get();

    gapWidth_ = dimensions.x / segments;

    rectangleWidth_ = gapWidth_ * 2;

    positions_.clear();

    auto drawValues = [this](const vec2& at, const float value, const float textBoxWidth) {
        auto text = value == std::numeric_limits<float>::max()
                        ? "inf"
                        : std::string{fmt::format("{:03.4e}", value)};
        auto bounds = nvgContext_.textBounds(at, text);

        nvgContext_.beginPath();
        nvgContext_.fontSize(static_cast<float>(fontProperties_.fontSize_.get()));
        nvgContext_.fontFace(fontProperties_.fontFace_.getSelectedIdentifier());
        nvgContext_.textAlign(NanoVGContext::Alignment::Center_Middle);
        nvgContext_.fillColor(fontColor_.get());
        nvgContext_.textBox(at, textBoxWidth, text);
        nvgContext_.closePath();
    };

    auto drawRectangle = [&, this](const size_t i, const vec2& extends) {
        const auto& position = positions_[i];
        auto& color = colorMap[i];

        auto flipY = [&](const vec2 pos) { return vec2{pos.x, dimensions.y - pos.y}; };

        /*
         * Unfortunately, picking has never been merged into nanovg. Therefore, we need to do it
         * ourselves. It'll be inaccurate since the rounded corners won't be respected but it'll do.
         */
        const auto hitPos = flipY(position);
        if (mousePos_.x >= hitPos.x && mousePos_.x <= hitPos.x + extends.x &&
            mousePos_.y <= hitPos.y && mousePos_.y >= hitPos.y - extends.y) {
            auto hcl = SegmentationColorHelper::rgbToHcl(dvec3(color));
            hcl.z *= 1.0f + luminanceMultiplier_.get();
            color = dvec4(
                glm::clamp(SegmentationColorHelper::hclToRgb(hcl), dvec3(0.0), dvec3(1.0)), 1.0);
        }
        
        nvgContext_.beginPath();
        nvgContext_.roundedRect(position, extends, cornerRadius_.get());
        
        if (selection.contains(i)) {
            nvgContext_.strokeColor(vec4{vec3{0.5f}, 1.0f});
            nvgContext_.strokeWidth(strokeWitdth_.get());
            nvgContext_.stroke();
        }
        nvgContext_.fillColor(color);
        nvgContext_.fill();
        nvgContext_.closePath();
    };

    for (size_t i{0}; i < iNumberOfSegments; ++i) {
        positions_.emplace_back((gapWidth_ * static_cast<float>(i + 1)) +
                                   (rectangleWidth_ * static_cast<float>(i)) + marginLeft_.get(),
                               dimensions.y - marginBottom_.get() - height_.get());
    }

    for (size_t i{0}; i < iNumberOfSegments; ++i) {
        drawRectangle(i, vec2{rectangleWidth_, height_.get()});
    }

    // Separeta loop so the text would always be on top
    for (size_t i{0}; i < iNumberOfSegments; ++i) {
        drawValues(positions_[i] + vec2(0, height_.get() / 2.0f), segmentMinima[i], rectangleWidth_);
    }

    nvgContext_.deactivate();

    utilgl::deactivateCurrentTarget();
}

}  // namespace inviwo
