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
#include <inviwo/core/datastructures/geometry/typedmesh.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/interaction/cameratrackball.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/core/properties/cameraproperty.h>
#include <modules/opengl/shader/shader.h>
#include <inviwo/core/interaction/pickingmapper.h>
#include <modules/brushingandlinking/ports/brushingandlinkingports.h>
#include <inviwo/core/util/colorbrewer.h>
#include <inviwo/core/properties/templateproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/boolproperty.h>

namespace inviwo {

/** \docpage{org.inviwo.ContourExplorerProcessor, Contour Explorer}
 * ![](org.inviwo.ContourExplorerProcessor.png?classIdentifier=org.inviwo.ContourExplorerProcessor)
 * Render an arbitrary slice of a volume in place, i.e. the slice will be
 * oriented as it would have been in the volume.
 *
 * ### Inports
 *   * __volume__ The input volume
 *   * __background__ Optional background image
 * ### Outports
 *   * __outport__ Rendered slice
 *
 * ### Properties
 *   * __Plane Normal__ Defines the normal of the plane in texture/data space
 * [0,1]
 *   * __Plane Position__ Defines a point in the plane in texture/data space
 * [0,1]
 *   * __Transfer Function__ Defines the transfer function for mapping voxel
 * values to color and opacity
 *   * __Camera__ Camera used for rendering
 *   * __Trackball__ Trackball for handling interaction
 */
class IVW_MODULE_CONTOUREXPLORER_API ContourExplorerProcessor : public Processor {
public:
    ContourExplorerProcessor();
    virtual ~ContourExplorerProcessor() = default;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void initializeResources() override;

   protected:
    virtual void process() override;
    void planeSettingsChanged();
    void handlePicking(PickingEvent* p);
    void updateTF();
    void generateIsoVolume();

   private:
    VolumeInport inport_;
    ImageInport backgroundPort_;
    ImageOutport outport_;
    VolumeOutport isoVolumeOutport_;
    BrushingAndLinkingInport brushingAndLinkingInport_;

    Shader shader_;

    BoolProperty ignoreZeroIndex_;

    FloatVec3Property planeNormal_;
    FloatVec3Property planePosition_;

    TemplateOptionProperty<colorbrewer::Family> family_;
    TransferFunctionProperty transferFunction_;

    ButtonProperty generateVolumeButton_;

    CameraProperty camera_;
    CameraTrackball trackball_;

    TypedMesh<buffertraits::PositionsBuffer> embeddedMesh_;
    PickingMapper picking_;
};

}  // namespace inviwo
