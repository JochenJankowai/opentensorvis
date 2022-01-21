/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2019 Inviwo Foundation
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

#include <inviwo/core/ports/outport.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/interaction/cameratrackball.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/cameraproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/rendering/meshdrawer.h>
#include <inviwo//geometrysilhouette/geometrysilhouettemoduledefine.h>
#include <modules/opengl/image/imagecompositor.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/shader/shader.h>
#include <modules/opengl/shader/shaderutils.h>
#include <map>

namespace inviwo {

/** \docpage{org.inviwo.GeometrySilhouette, Geometry Silhouette}
 * ![](org.inviwo.GeometrySilhouette.png?classIdentifier=org.inviwo.GeometrySilhouette)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __inport__ Mesh of which the silhouette should be drawn. Supports multiple inputs.
 *   * __imageInport__ Optional image into which the silhouette will be drawn.
 *
 * ### Outports
 *   * __outport__ Output image.
 *
 * ### Properties
 *   * __overrideColor__ Switch color overriding on/off.
 *   * __colour__ Override color.
 *   * __binary__ Switch binary silhouette rendering on/off.
 *   * __threshold__ Alpha threshold for binary silhouette rendering. If the alpha value in the
 * original silhouette > threshold it will be set to 1, 0 otherwise.
 *   * __showBackfaces__ Switch backfaces on/off.
 *   * __widenNarrow__ Sets the exponent to the interpolation method, e.g. widens/narrows the gradient.
 *   * __easeMethod__ Sets the easing method. Ease in is defined as y=1-cos(x*PI/2), ease out is defined as y=sin(x*PI/2).
 */

/**
 * \class GeometrySilhouette
 * \brief VERY_BRIEFLY_DESCRIBE_THE_PROCESSOR
 * DESCRIBE_THE_PROCESSOR_FROM_A_DEVELOPER_PERSPECTIVE
 */
class IVW_MODULE_GEOMETRYSILHOUETTE_API GeometrySilhouette : public Processor {
public:
    GeometrySilhouette();
    virtual ~GeometrySilhouette() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

protected:
    using DrawerMap = std::multimap<const Outport*, std::unique_ptr<MeshDrawer>>;

    // Ports
    MeshFlatMultiInport inport_;
    ImageInport image_;
    ImageOutport outport_;

    // Members
    Shader shader_;
    DrawerMap drawers_;

    // Properties
    CameraProperty camera_;
    CameraTrackball tb_;
    BoolProperty overrideColor_;
    FloatVec4Property colour_;
    BoolProperty binary_;
    FloatProperty threshold_;
    FloatProperty opacity_;
    BoolProperty showBackfaces_;
    BoolProperty enableDepthTest_;
    FloatProperty widenNarrow_;
    CompositeProperty settings_;
    OptionPropertyInt easeMethod_;

    // Methods
    void updateDrawers();
};

}  // namespace inviwo
