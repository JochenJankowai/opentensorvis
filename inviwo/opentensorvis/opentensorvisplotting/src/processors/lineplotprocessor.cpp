/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2020 Inviwo Foundation
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

#include <inviwo/opentensorvisplotting/processors/lineplotprocessor.h>
#include <inviwo/core/util/foreacharg.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/util/zip.h>
#include <inviwo/core/util/rendercontext.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/datastructures/geometry/mesh.h>
#include <inviwo/core/datastructures/geometry/typedmesh.h>

#include <modules/opengl/rendering/meshdrawergl.h>

#include <modules/opengl/openglutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/texture/textureutils.h>

#include <algorithm>

#include "inviwo/core/interaction/events/pickingevent.h"
#include "inviwo/nanovgutils/nanovgutilsmodule.h"

#include <inviwo/core/interaction/events/mouseevent.h>

#include <inviwo/opentensorvisbase/util/misc.h>

#include "modules/fontrendering/fontrenderingmodule.h"
#include "modules/fontrendering/util/fontutils.h"

namespace inviwo {

namespace plot {

const std::string LineAxisProperty::classIdentifier = "org.inviwo.LineAxisProperty";
std::string LineAxisProperty::getClassIdentifier() const { return classIdentifier; }

LineAxisProperty::LineAxisProperty(std::string identifier, std::string displayName,
                                   size_t firstColIndex, bool vertical,
                                   InvalidationLevel invalidationLevel, PropertySemantics semantics)
    : BoolCompositeProperty{identifier, displayName, true, invalidationLevel, semantics}
    , column_{"column", "Column", false, firstColIndex}
    , color_{"lineColor", "Line Color", util::ordinalColor(0.11f, 0.42f, 0.63f)}
    , style_{"axisStyle", "Style",
             vertical ? AxisProperty::Orientation::Vertical : AxisProperty::Orientation::Horizontal}
    , axisRenderer_{style_} {

    util::for_each_in_tuple([&](auto& e) { this->addProperty(e); }, props());
    if (vertical) {
        style_.flipped_.set(true);
        style_.captionSettings_.setChecked(true);
        style_.setCaption(displayName);
    }
    style_.setCurrentStateAsDefault();

    column_.onChange([this]() { style_.setCaption(column_.getColumnHeader()); });
}

LineAxisProperty::LineAxisProperty(const LineAxisProperty& rhs)
    : BoolCompositeProperty{rhs}
    , column_{rhs.column_}
    , color_{rhs.color_}
    , style_{rhs.style_}
    , axisRenderer_{style_} {
    util::for_each_in_tuple([&](auto& e) { this->addProperty(e); }, props());
    column_.onChange([this]() { style_.setCaption(column_.getColumnHeader()); });
}

auto LineAxisProperty::clone() const -> LineAxisProperty* {
    RenderContext::getPtr()->activateDefaultRenderContext();
    return new LineAxisProperty(*this);
}
}  // namespace plot

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LinePlotProcessor::processorInfo_{
    "org.inviwo.LinePlotProcessor",  // Class identifier
    "Line Plot",                     // Display name
    "OpenTensorVis",                 // Category
    CodeState::Experimental,         // Code state
    Tags::GL,                        // Tags
};
const ProcessorInfo LinePlotProcessor::getProcessorInfo() const { return processorInfo_; }

LinePlotProcessor::LinePlotProcessor()
    : Processor{}
    , dataFramePort_{"dataFrame_"}
    , brushingPort_{"brushing"}
    , bgInport_{"bg"}
    , outport_{"outport"}

    , axisStyle_{"axisStyle", "Global Axis Style"}
    , xAxis_{"xAxis", "X-axis", 0, false}
    , yAxisList_{"yaxisList", "Y Axes",
                 std::make_unique<plot::LineAxisProperty>("yaxis1", "Y Axis 1", 2, true)}

    , dataPoints_{"dataPoints", "Data Points", false}
    , pointRadius_{"pointRadius", "Radius", 5, 0, 10, 0.01f}
    , borderWidth_{"borderWidth", "Border Width", 2, 0, 20}
    , borderColor_{"borderColor", "Border Color", util::ordinalColor(0, 0, 0, 1)}
    , useCircle_("useCircle", "Use Circles (else squares)", true)

    , hoverColor_{"hoverColor", "Hover Color", util::ordinalColor(1.0f, 0.906f, 0.612f, 1)}
    , selectionColor_{"selectionColor", "Selection Color",
                      util::ordinalColor(1.0f, 0.769f, 0.247f, 1)}
    , margins_{"margins", "Margins", 10.0f, 10.0f, 30.0f, 100.0f}
    , axisMargin_{"axisMargin", "Axis Margin", 15.0f, 0.0f, 50.0f}
    , axisSpacing_{"axisSpacing", "Axis Spacing", 50, 0, 200}
    , hovering_{"hovering", "Enable Hovering", true}
    , lineSettings_{"lineSettings", "Line Settings"}
    , syncColorsWithCaptions_{"syncColorsWithCaptions", "Sync Captions with Line Colors",
                              [this]() {
                                  for (auto p : yAxisList_) {
                                      if (auto lineProp =
                                              dynamic_cast<plot::LineAxisProperty*>(p)) {
                                          lineProp->style_.captionSettings_.color_.set(
                                              lineProp->color_.get());
                                      }
                                  }
                              }}
    , splitColumn_{"splitColumn", "Split Column", dataFramePort_, true}
    , probeProperties_{"probeProperties", "Probe properties"}
    , useProbe_{"useProbe", "Enable probe", true}
    , probeWidth_{"probeWidth", "Width", 1.0f, 0.1f, 4.0f, 0.1f}
    , probeColor_{"probeColor", "Color", vec4{0, 0, 0, 1}}
    , probeDashed_{"probeDashed", "Dash probe", true}
    , probeDashWidth_{"probeDashWidth", "Dash width", 1.0f, 0.1f, 40.0f, 0.1f}
    , probeGapWidth_{"probeGapWidth", "Gap width", 1.0f, 0.1f, 40.0f, 0.1f}
    , probeFontProperties_{"probeFontProperties", "Font properties"}
    , lineRenderer_{&lineSettings_}
    , pointShader_{"lineplotdot.vert", "scatterplot.geom", "scatterplot.frag"}
    , picking_{this, 1, [this](PickingEvent* p) { objectPicked(p); }}
    , camera_{vec3{0.5f, 0.5f, 1.0f},
              vec3{0.5f, 0.5f, 0.0f},
              vec3{0.0f, 1.0f, 0.0f},
              0.1f,
              2.0f,
              1.0f}
    , mouseMove_(
          "mouseMove", "Mouse Move", [this](Event* e) { mouseMoveEvent(e); },
          MouseButtons(flags::any), MouseState::Move)
    , ndc_(0)
    , nvgContext_{
          InviwoApplication::getPtr()->getModuleByType<NanoVGUtilsModule>()->getNanoVGContext()} {

    addPort(dataFramePort_);
    addPort(brushingPort_);
    addPort(bgInport_);
    addPort(outport_);

    bgInport_.setOptional(true);

    dataPoints_.addProperties(pointRadius_, borderWidth_, borderColor_, useCircle_);
    dataPoints_.setCollapsed(true);
    dataPoints_.setCurrentStateAsDefault();

    lineSettings_.lineWidth_.set(2.0f);
    lineSettings_.pseudoLighting_.set(false);
    lineSettings_.roundDepthProfile_.set(false);
    lineSettings_.setCollapsed(true);
    lineSettings_.setCurrentStateAsDefault();

    probeProperties_.addProperties(useProbe_, probeWidth_, probeColor_, probeDashed_,
                                   probeDashWidth_, probeGapWidth_, probeFontProperties_);

    addProperties(axisStyle_, xAxis_, yAxisList_, syncColorsWithCaptions_, dataPoints_, hoverColor_,
                  selectionColor_, margins_, axisMargin_, axisSpacing_, hovering_, lineSettings_,
                  splitColumn_, mouseMove_, probeProperties_);

    axisStyle_.registerProperty(xAxis_.style_);
    axisStyle_.setCollapsed(true);
    axisStyle_.labelFormat_.set("%g");
    axisStyle_.setCurrentStateAsDefault();

    xAxis_.color_.setVisible(false);
    yAxisList_.PropertyOwnerObservable::addObserver(this);

    dataFramePort_.onChange([this]() {
        xAxis_.column_.setOptions(dataFramePort_.getData());
        for (auto p : yAxisList_) {
            if (auto lineProp = dynamic_cast<plot::LineAxisProperty*>(p)) {
                lineProp->column_.setOptions(dataFramePort_.getData());
            }
        }
    });
    xAxis_.onChange([this]() { onAxisChange(&xAxis_); });
    splitColumn_.onChange([this]() { meshDirty_ = true; });
    // add first y axis
    yAxisList_.constructProperty(0);

    pointShader_.onReload([this]() { invalidate(InvalidationLevel::InvalidOutput); });

    auto loadFontIfNeeded = [this]() {
        auto fontFace = probeFontProperties_.fontFace_.getSelectedValue();
        auto fontName = probeFontProperties_.fontFace_.getSelectedIdentifier();

        if (nvgContext_.findFont(fontFace) == -1) {
            if (nvgContext_.createFont(fontName, fontFace) == -1) {
                LogError(fmt::format("Could not create font: {}", fontFace));
            }
        }

        invalidate(InvalidationLevel::InvalidOutput);
    };

    loadFontIfNeeded();

    probeFontProperties_.fontFace_.onChange(loadFontIfNeeded);
}

void LinePlotProcessor::mouseMoveEvent(Event* event) {
    if (auto mouseEvent = event->getAs<MouseEvent>()) {
        auto p = mouseEvent->posNormalized();

        ndc_ = glm::clamp(p, dvec2(0.0), dvec2(1.0));

        invalidate(InvalidationLevel::InvalidOutput);
    }
}

void LinePlotProcessor::process() {
    utilgl::activateTargetAndClearOrCopySource(outport_, bgInport_);
    utilgl::DepthFuncState depthfunc(GL_ALWAYS);

    const auto dims = outport_.getDimensions();
    const ivec2 lowerLeft(margins_.getLeft(), margins_.getBottom());
    const ivec2 upperRight(dims.x - 1 - margins_.getRight(), dims.y - 1 - margins_.getTop());

    const int padding = static_cast<int>(axisMargin_.get());

    // draw horizontal axis
    xAxis_.axisRenderer_.render(dims, lowerLeft + ivec2(padding, 0),
                                ivec2(upperRight.x - padding, lowerLeft.y));
    // draw vertical axes
    int xOffset = 0;
    for (auto p : yAxisList_) {
        if (auto lineProp = dynamic_cast<plot::LineAxisProperty*>(p);
            lineProp && lineProp->isChecked()) {
            lineProp->axisRenderer_.render(dims, lowerLeft + ivec2(xOffset, padding),
                                           ivec2(lowerLeft.x + xOffset, upperRight.y - padding));
            xOffset -= axisSpacing_;
        }
    }

    if (meshDirty_) {
        mesh_ = createLines();
        meshDirty_ = false;
    }

    if (mesh_) {
        // restrict line rendering to plotting area
        {
            const ivec2 plotDims{upperRight.x - lowerLeft.x - 2 * padding,
                                 upperRight.y - lowerLeft.y - 2 * padding};
            utilgl::ViewportState viewport(lowerLeft.x + padding, lowerLeft.y + padding, plotDims.x,
                                           plotDims.y);
            utilgl::BlendModeState blending(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            lineRenderer_.render(*mesh_.get(), camera_, plotDims, &lineSettings_);

            if (dataPoints_.isChecked()) {
                drawDataPoints(plotDims);
            }
        }

        if (useProbe_.get()) {
            drawProbe(dims);
        }
    }

    utilgl::deactivateCurrentTarget();
}

void inviwo::LinePlotProcessor::drawProbe(const ivec2& dims) {
    auto drawDashedX = [this](const vec2& from, const vec2& to) {
        float offset = from.x;

        while (offset <= to.x) {
            nvgContext_.moveTo(vec2{offset, to.y});
            offset += probeDashWidth_.get();
            nvgContext_.lineTo(vec2(std::min(offset, to.x), to.y));
            offset += probeGapWidth_.get();
        }

        nvgContext_.strokeColor(probeColor_.get());
        nvgContext_.strokeWidth(probeWidth_.get());
        nvgContext_.stroke();
        nvgContext_.closePath();
    };

    auto drawDashedY = [this](const vec2& from, const vec2& to) {
        float offset = from.y;

        while (offset <= to.y) {
            nvgContext_.moveTo(vec2{from.x, offset});
            offset += probeDashWidth_.get();
            nvgContext_.lineTo(vec2{from.x, std::min(offset, to.y)});
            offset += probeGapWidth_.get();
        }

        nvgContext_.strokeColor(probeColor_.get());
        nvgContext_.strokeWidth(probeWidth_.get());
        nvgContext_.stroke();
        nvgContext_.closePath();
    };

    auto drawSolid = [this](const vec2& from, const vec2& to) {
        nvgContext_.moveTo(from);
        nvgContext_.lineTo(to);
    };

    auto calculateValues = []() -> vec2 {

    };

    auto drawValues = [this](const vec2& at, const vec2& values) {
        auto text = std::string{fmt::format("[{:03.4f}, {:03.4f}]", values.x, values.y)};
        auto bounds = nvgContext_.textBounds(at, text);

        nvgContext_.beginPath();
        nvgContext_.fontSize(static_cast<float>(probeFontProperties_.fontSize_.get()));
        nvgContext_.fontFace(probeFontProperties_.fontFace_.getSelectedIdentifier());
        nvgContext_.textAlign(NanoVGContext::Alignment::Center_Bottom);
        nvgContext_.fillColor(probeColor_.get());
        nvgContext_.textBox(at, bounds[2] - bounds[0], text);
        nvgContext_.closePath();
    };

    const auto denormalizedMousePositionX = tensorutil::denormalizeValue(
        static_cast<float>(ndc_.x), 0.0f, static_cast<float>(dims.x - 1));
    const auto denormalizedMousePositionY = tensorutil::denormalizeValue(
        static_cast<float>(ndc_.y), 0.0f, static_cast<float>(dims.y - 1));

    const vec2 lowerLeft(margins_.getLeft(), margins_.getBottom());
    const vec2 upperRight(dims.x - 1 - margins_.getRight(), dims.y - 1 - margins_.getTop());
    const auto padding = axisMargin_.get();

    const auto howFarInX = denormalizedMousePositionX - lowerLeft.x - padding;
    const auto width = upperRight.x - lowerLeft.x - 2.0f * padding;
    const auto howFarInY = denormalizedMousePositionY - lowerLeft.y - padding;
    const auto height = upperRight.y - lowerLeft.y - 2.0f * padding;

    const vec2 normalizedMouseCoordsRelativeToViewport{
        static_cast<float>(howFarInX) / static_cast<float>(width),
        static_cast<float>(howFarInY) / static_cast<float>(height)};

    if (denormalizedMousePositionX > upperRight.x - padding ||
        denormalizedMousePositionX < lowerLeft.x+padding ||
        denormalizedMousePositionY > upperRight.y - padding ||
        denormalizedMousePositionY < lowerLeft.y+padding) {
        return;
    }

    nvgContext_.activate(dims);

    auto from = vec2{lowerLeft.x, dims.y - denormalizedMousePositionY};
    auto to = vec2{upperRight.x - padding, dims.y - denormalizedMousePositionY};

    nvgContext_.beginPath();
    if (probeDashed_.get()) {
        drawDashedX(from, to);
    } else {
        drawSolid(from, to);
    }
    nvgContext_.strokeColor(probeColor_.get());
    nvgContext_.strokeWidth(probeWidth_.get());
    nvgContext_.stroke();
    nvgContext_.closePath();

    from = vec2{denormalizedMousePositionX, dims.y - lowerLeft.y};
    to = vec2{denormalizedMousePositionX, dims.y - upperRight.y + padding};

    nvgContext_.beginPath();
    if (probeDashed_.get()) {
        drawDashedY(to, from);
    } else {
        drawSolid(from, to);
    }
    nvgContext_.strokeColor(probeColor_.get());
    nvgContext_.strokeWidth(probeWidth_.get());
    nvgContext_.stroke();
    nvgContext_.closePath();

    const auto xRange = vec2(xAxis_.style_.getRange());
    const auto xVal = xRange.y * normalizedMouseCoordsRelativeToViewport.x;

    const auto yRange = vec2(dynamic_cast<plot::LineAxisProperty*>(yAxisList_.getProperties().front())->style_.getRange());
    const auto yVal = ((yRange.y-yRange.x) * normalizedMouseCoordsRelativeToViewport.y)+yRange.x;

    drawValues(vec2{denormalizedMousePositionX, dims.y - denormalizedMousePositionY},
               vec2{xVal, yVal});

    nvgContext_.deactivate();
}

void LinePlotProcessor::onDidAddProperty(Property* property, size_t) {
    NetworkLock lock(property);

    auto p = static_cast<plot::LineAxisProperty*>(property);
    axisStyle_.registerProperty(p->style_);

    p->style_.setCaption(p->getDisplayName());
    p->column_.onChange([this, p]() { onAxisChange(p); });
    p->column_.setOptions(dataFramePort_.getData());

    p->getBoolProperty()->onChange([this]() { meshDirty_ = true; });
    p->color_.onChange([this]() { meshDirty_ = true; });
    p->style_.range_.onChange([this]() { meshDirty_ = true; });

    invalidate(InvalidationLevel::InvalidOutput);
}

void LinePlotProcessor::onWillRemoveProperty(Property* property, size_t) {
    axisStyle_.unregisterProperty(*static_cast<plot::AxisProperty*>(property));
    invalidate(InvalidationLevel::InvalidOutput);
}

void LinePlotProcessor::objectPicked(PickingEvent* p) {}

void LinePlotProcessor::onAxisChange(plot::LineAxisProperty* p) {
    if (dataFramePort_.hasData()) {
        auto data = dataFramePort_.getData();

        auto col = data->getColumn(p->column_);
        IVW_ASSERT(col, "column not found after axis change");

        auto minmax = util::bufferMinMax(col->getBuffer().get(), IgnoreSpecialValues::Yes);
        p->style_.setRange(dvec2{minmax.first.x, minmax.second.x});

        meshDirty_ = true;
    }
}

std::shared_ptr<Mesh> LinePlotProcessor::createLines() {
    if (!dataFramePort_.hasData()) {
        return {};
    }

    using LineMesh = TypedMesh<buffertraits::PositionsBuffer1D, buffertraits::ColorsBuffer,
                               buffertraits::PickingBuffer>;

    //
    // TODO: reuse x axis values in separate vertex attribute and custom shader?
    //
    // TODO: filter points based on x axis
    //
    // TODO: sorting values or use sorting from DataFrame?
    //

    auto mesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::StripAdjacency);
    auto dataframe = dataFramePort_.getData();

    const auto lineRowIndices = [this]() -> std::vector<std::vector<size_t>> {
        if (splitColumn_.getSelectedValue() > -1) {
            const auto buffer =
                splitColumn_.getColumn()->getBuffer()->getRepresentation<BufferRAM>();
            return buffer->dispatch<std::vector<std::vector<size_t>>, dispatching::filter::Scalars>(
                [](auto buf) {
                    using ValueType = util::PrecisionValueType<decltype(buf)>;
                    std::vector<std::vector<size_t>> rows;
                    std::map<ValueType, size_t> valueIndex;
                    for (auto&& [row, v] : util::enumerate(buf->getDataContainer())) {
                        if (auto it = valueIndex.find(v); it != valueIndex.end()) {
                            rows[it->second].push_back(row);
                        } else {
                            valueIndex[v] = rows.size();
                            rows.emplace_back(1, row);
                        }
                    }
                    return rows;
                });
        } else {
            std::vector<size_t> rows(dataFramePort_.getData()->getNumberOfRows());
            std::iota(rows.begin(), rows.end(), 0u);
            return {std::move(rows)};
        }
    }();

    auto getValues = [](const plot::LineAxisProperty& p) {
        const vec2 range{p.style_.getRange()};
        const auto buffer = p.column_.getColumn()->getBuffer()->getRepresentation<BufferRAM>();

        return buffer->dispatch<std::vector<float>, dispatching::filter::Scalars>(
            [range](auto buf) {
                auto& data = buf->getDataContainer();
                std::vector<float> dst(data.size(), 0.0f);
                std::transform(data.begin(), data.end(), dst.begin(), [range](auto v) {
                    return (static_cast<float>(v) - range.x) / (range.y - range.x);
                });
                return dst;
            });
    };

    auto xpos = getValues(xAxis_);

    const size_t numValues = xpos.size() * yAxisList_.size();

    std::vector<vec2> positions;
    positions.reserve(numValues);
    std::vector<vec4> colors;
    colors.reserve(numValues);

    for (const auto& rows : lineRowIndices) {
        for (const auto p : yAxisList_) {
            if (const auto lineProp = dynamic_cast<const plot::LineAxisProperty*>(p);
                lineProp && lineProp->isChecked()) {
                const auto startIndex = positions.size();

                std::for_each(rows.begin(), rows.end(),
                              [&positions, xpos, ypos = getValues(*lineProp)](auto row) {
                                  positions.emplace_back(xpos[row], ypos[row]);
                              });

                const auto endIndex = positions.size();
                colors.resize(colors.size() + endIndex - startIndex, lineProp->color_);

                std::vector<uint32_t> indices(endIndex - startIndex + 2);
                indices.front() = static_cast<uint32_t>(startIndex);
                std::iota(indices.begin() + 1, indices.end(), indices.front());
                indices.back() = indices.back() - 1;
                mesh->addIndices(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::StripAdjacency),
                                 util::makeIndexBuffer(std::move(indices)));
            }
        }
    }

    mesh->addBuffer(BufferType::PositionAttrib, util::makeBuffer(std::move(positions)));
    mesh->addBuffer(BufferType::ColorAttrib, util::makeBuffer(std::move(colors)));

    return mesh;
}

void LinePlotProcessor::drawDataPoints(ivec2 plotDims) {
    pointShader_.activate();
    const vec2 pixelSize = vec2(1) / vec2(plotDims);
    pointShader_.setUniform("radius", pointRadius_.get());
    pointShader_.setUniform("borderWidth", borderWidth_.get());
    pointShader_.setUniform("borderColor", borderColor_.get());
    pointShader_.setUniform("pixelSize", pixelSize);
    pointShader_.setUniform("dims", ivec2(plotDims));
    pointShader_.setUniform("circle", useCircle_.get() ? 1 : 0);
    pointShader_.setUniform("pickingEnabled", picking_.isEnabled());

    MeshDrawerGL::DrawObject drawer(mesh_->getRepresentation<MeshGL>(),
                                    mesh_->getDefaultMeshInfo());
    utilgl::setShaderUniforms(pointShader_, *mesh_.get(), "geometry");

    drawer.draw(MeshDrawerGL::DrawMode::Points);

    pointShader_.deactivate();
}

}  // namespace inviwo