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

#pragma once

#include <inviwo/opentensorvisplotting/opentensorvisplottingmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/boolcompositeproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/interaction/pickingmapper.h>
#include <inviwo/core/datastructures/camera/orthographiccamera.h>

#include <modules/basegl/rendering/linerenderer.h>
#include <modules/basegl/datastructures/linesettings.h>
#include <modules/basegl/properties/linesettingsproperty.h>

#include <modules/opengl/shader/shader.h>

#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>
#include <modules/brushingandlinking/ports/brushingandlinkingports.h>

#include <modules/plotting/properties/marginproperty.h>
#include <modules/plotting/properties/axisproperty.h>
#include <modules/plotting/properties/axisstyleproperty.h>
#include <modules/plottinggl/utils/axisrenderer.h>

#include <inviwo/core/properties/eventproperty.h>

#include <inviwo/nanovgutils/nanovgcontext.h>

#include <modules/fontrendering/properties/fontproperty.h>

namespace inviwo {

class Mesh;

namespace plot {

class LineAxisProperty : public BoolCompositeProperty {
public:
    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;

    LineAxisProperty(std::string identifier, std::string displayName, size_t firstColIndex = 0,
                     bool vertical = true,
                     InvalidationLevel invalidationLevel = InvalidationLevel::InvalidOutput,
                     PropertySemantics semantics = PropertySemantics::Default);

    LineAxisProperty(const LineAxisProperty& rhs);
    virtual LineAxisProperty* clone() const override;
    virtual ~LineAxisProperty() = default;

    DataFrameColumnProperty column_;
    FloatVec4Property color_;
    AxisProperty style_;

    AxisRenderer axisRenderer_;

private:
    auto props() { return std::tie(column_, color_, style_); }
    auto props() const { return std::tie(column_, color_, style_); }
};

}  // namespace plot
/** \docpage{org.inviwo.LinePlotProcessor, Line Plot}
 * ![](org.inviwo.LinePlotProcessor.png?classIdentifier=org.inviwo.LinePlotProcessor)
 * This processor provides the functionality to generate a line plot from a DataFrame.
 *
 * ### Inports
 *   * __DataFrame__  data input for plotting
 *   * __BrushingAndLinking__   inport for brushing & linking interactions
 *
 * ### Outports
 *   * __outport__   rendered image of the scatter plot
 *
 */
class IVW_MODULE_OPENTENSORVISPLOTTING_API LinePlotProcessor : public Processor,
                                                               public PropertyOwnerObserver {
public:
    LinePlotProcessor();
    virtual ~LinePlotProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    // virtual void invokeEvent(Event* event) override;
private:
    void objectPicked(PickingEvent* p);
    void mouseMoveEvent(Event* event);

    virtual void onDidAddProperty(Property* property, size_t index) override;
    virtual void onWillRemoveProperty(Property* property, size_t index) override;

    void onAxisChange(plot::LineAxisProperty* p);

    std::shared_ptr<Mesh> createLines();
    void drawDataPoints(ivec2 plotDims);

    void drawProbe(const ivec2& dims);

    DataFrameInport dataFramePort_;
    BrushingAndLinkingInport brushingPort_;
    ImageInport bgInport_;
    ImageOutport outport_;

    plot::AxisStyleProperty axisStyle_;

    plot::LineAxisProperty xAxis_;
    ListProperty yAxisList_;

    BoolCompositeProperty dataPoints_;
    FloatProperty pointRadius_;
    FloatProperty borderWidth_;
    FloatVec4Property borderColor_;
    BoolProperty useCircle_;

    FloatVec4Property hoverColor_;
    FloatVec4Property selectionColor_;
    plot::MarginProperty margins_;
    FloatProperty axisMargin_;
    IntProperty axisSpacing_;

    BoolProperty hovering_;
    LineSettingsProperty lineSettings_;
    ButtonProperty syncColorsWithCaptions_;
    DataFrameColumnProperty splitColumn_;

    CompositeProperty probeProperties_;
    BoolProperty useProbe_;
    BoolProperty xProbe_;
    BoolProperty yProbe_;
    FloatProperty probeWidth_;
    FloatVec4Property probeColor_;
    BoolProperty probeDashed_;
    FloatProperty probeDashWidth_;
    FloatProperty probeGapWidth_;
    FontProperty probeFontProperties_;

    algorithm::LineRenderer lineRenderer_;
    Shader pointShader_;

    PickingMapper picking_;
    OrthographicCamera camera_;

    std::shared_ptr<Mesh> mesh_;

    bool meshDirty_ = false;

    EventProperty mouseMove_;
    dvec2 ndc_;
    NanoVGContext &nvgContext_;
};

}  // namespace inviwo