/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2020 Inviwo Foundation
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

#include <inviwo/opentensorvisbase/opentensorvisbasemodule.h>

#include <modules/opengl/shader/shadermanager.h>

#include <inviwo/opentensorvisbase/datavisualizer/hyperlicvisualizer2d.h>
#include <inviwo/opentensorvisbase/datavisualizer/hyperlicvisualizer3d.h>
#include <inviwo/opentensorvisbase/datavisualizer/anisotropyraycastingvisualizer.h>

#include <inviwo/opentensorvisbase/ports/tensorfieldport.h>
#include <inviwo/opentensorvisbase/processors/hyperstreamlines.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dmetadata.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dsubsample.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dsubset.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dtoimage.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dfiberangle.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dbasismanipulation.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dboundingbox.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dmasktovolume.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dmetadata.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dsubsample.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dsubset.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dgenerator.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dlic.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dslice.h>
#include <inviwo/opentensorvisbase/processors/tensorfield2dasrgba.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dtovolume.h>
#include <inviwo/opentensorvisbase/processors/tensorfield3dinformation.h>
#include <inviwo/opentensorvisbase/properties/eigenvalueproperty.h>

namespace inviwo {

OpenTensorVisBaseModule::OpenTensorVisBaseModule(InviwoApplication* app)
    : InviwoModule(app, "OpenTensorVisBase") {
    ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    registerPort<TensorField2DInport>();
    registerPort<TensorField2DOutport>();
    registerPort<TensorField3DInport>();
    registerPort<TensorField3DOutport>();

    registerProcessor<HyperStreamlines>();
    registerProcessor<TensorField2DMetaData>();
    registerProcessor<TensorField2DSubsample>();
    registerProcessor<TensorField2DSubset>();
    registerProcessor<TensorField2DToImage>();
    registerProcessor<TensorField3DFiberAngle>();
    registerProcessor<TensorField3DBasisManipulation>();
    registerProcessor<TensorField3DBoundingBox>();
    registerProcessor<TensorField3DMaskToVolume>();
    registerProcessor<TensorField3DMetaData>();
    registerProcessor<TensorField3DSubsample>();
    registerProcessor<TensorField3DSubset>();
    registerProcessor<TensorField2DGenerator>();
    registerProcessor<TensorField2DLIC>();
    registerProcessor<TensorField3DSlice>();
    registerProcessor<TensorField2DAsRGBA>();
    registerProcessor<TensorField3DToVolume>();
    registerProcessor<TensorField3DInformation>();

    registerProperty<EigenValueProperty>();

    registerDataVisualizer(std::make_unique<HyperLICVisualizer2D>(app));
    registerDataVisualizer(std::make_unique<HyperLICVisualizer3D>(app));
    registerDataVisualizer(std::make_unique<AnisotropyRaycastingVisualizer>(app));
}

}  // namespace inviwo
