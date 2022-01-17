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
 * 1. Redistributions of source code must retain the above copyright notice,
 *this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/contourexplorer/processors/contourexplorerprocessor.h>
#include <inviwo/core/algorithm/boundingbox.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/algorithm/cubeplaneintersection.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/datastructures/geometry/mesh.h>
#include <inviwo/core/datastructures/geometry/plane.h>
#include <inviwo/core/interaction/events/pickingevent.h>
#include <modules/opengl/rendering/meshdrawergl.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/volume/volumeutils.h>
#include <inviwo/contourexplorer/algorithm/generatesegmentedtf.h>
#include <inviwo/core/network/networklock.h>
#include <set>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo ContourExplorerProcessor::processorInfo_{
    "org.inviwo.ContourExplorerProcessor",  // Class identifier
    "Contour Explorer",                     // Display name
    "OpenTensorVis",                        // Category
    CodeState::Experimental,                // Code state
    Tags::GL,                               // Tags
};
const ProcessorInfo ContourExplorerProcessor::getProcessorInfo() const { return processorInfo_; }

ContourExplorerProcessor::ContourExplorerProcessor()
    : Processor()
    , inport_{"volume"}
    , backgroundPort_{"background"}
    , outport_{"outport"}
    , isoVolumeOutport_("isoVolumeOutport")
    , brushingAndLinkingInport_{"brushingAndLinkingInport"}
    , shader_{"embeddedvolumeslice.vert", "embeddedvolumeslice.frag", Shader::Build::No}
    , ignoreZeroIndex_("ignoreZeroIndex", "Ignore zero index", true)
    , planeNormal_{"planeNormal",          "Plane Normal",      vec3(1.f, 0.f, 0.f),
                   vec3(-1.f, -1.f, -1.f), vec3(1.f, 1.f, 1.f), vec3(0.01f, 0.01f, 0.01f)}
    , planePosition_{"planePosition", "Plane Position", vec3(0.5f), vec3(0.0f), vec3(1.0f)}
    , transferFunction_{"transferFunction", "Transfer Function", &inport_}
    , generateVolumeButton_("generateVolumeButton", "Generate iso volume", InvalidationLevel::Valid)
    , camera_{"camera", "Camera", util::boundingBox(inport_)}
    , trackball_{&camera_}
    , picking_{this, 1, [this](PickingEvent* e) { handlePicking(e); }} {
    addPort(inport_);
    addPort(backgroundPort_).setOptional(true);
    addPort(brushingAndLinkingInport_);
    addPorts(outport_, isoVolumeOutport_);

    inport_.onChange([this]() { updateTF(); });

    transferFunction_.setReadOnly(true);

    generateVolumeButton_.onChange([this]() { generateIsoVolume(); });

    addProperties(ignoreZeroIndex_, planeNormal_, planePosition_, transferFunction_,
                  generateVolumeButton_, camera_, trackball_);

    planePosition_.onChange([this]() { planeSettingsChanged(); });
    planeNormal_.onChange([this]() { planeSettingsChanged(); });
    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });
}

void ContourExplorerProcessor::initializeResources() {
    auto vso = shader_.getVertexShaderObject();
    vso->clearInDeclarations();
    vso->addInDeclaration("in_Vertex", buffertraits::PositionsBuffer::bi().location, "vec3");
    shader_.build();
    planeSettingsChanged();
}

void ContourExplorerProcessor::planeSettingsChanged() {
    if (!inport_.hasData()) return;

    // In texture space
    const Plane plane{*planePosition_, glm::normalize(*planeNormal_)};

    auto& intersections =
        embeddedMesh_.getEditableVertices()->getEditableRAMRepresentation()->getDataContainer();
    intersections.clear();

    if (embeddedMesh_.getNumberOfIndicies() == 0) {
        embeddedMesh_.addIndexBuffer(DrawType::Triangles, ConnectivityType::None);
    }

    auto& inds = embeddedMesh_.getIndices(0)->getEditableRAMRepresentation()->getDataContainer();
    inds.clear();

    util::cubePlaneIntersectionAppend(plane, intersections, inds);

    embeddedMesh_.setModelMatrix(inport_.getData()->getModelMatrix());
    embeddedMesh_.setWorldMatrix(inport_.getData()->getWorldMatrix());
}

void ContourExplorerProcessor::handlePicking(PickingEvent* p) {
    if (inport_.hasData() && p->getPressItems().contains(PickingPressItem::Primary) &&
        p->getHoverState() == PickingHoverState::None) {

        const auto data = inport_.getData();

        const Plane plane{*planePosition_, glm::normalize(*planeNormal_)};
        const auto& ct = data->getCoordinateTransformer(camera_);

        const vec3 ndc1{p->getNDC().x, p->getNDC().y, -1.0};
        const auto worldPos1 = camera_.getWorldPosFromNormalizedDeviceCoords(ndc1);
        const auto dataPos1 = vec3{ct.getWorldToDataMatrix() * vec4{worldPos1, 1.0f}};

        const vec3 ndc2{p->getNDC().x, p->getNDC().y, 1.0};
        const auto worldPos2 = camera_.getWorldPosFromNormalizedDeviceCoords(ndc2);
        const auto dataPos2 = vec3{ct.getWorldToDataMatrix() * vec4{worldPos2, 1.0f}};

        uint32_t value{};

        if (const auto dataPoint = plane.getIntersection(dataPos1, dataPos2); dataPoint) {
            const auto index = static_cast<size3_t>(
                vec3{ct.getDataToIndexMatrix() * vec4{*dataPoint, 1.0f}} + vec3{0.5f});

            const auto cind = glm::clamp(index, size3_t{0}, data->getDimensions() - size3_t{1});
            value = static_cast<uint32_t>(data->getRepresentation<VolumeRAM>()->getAsDVec4(cind).x);

            if (ignoreZeroIndex_.get() && value == 0) {
            } else {
                auto selection = brushingAndLinkingInport_.getSelectedIndices();
                if (selection.contains(value)) {
                    selection.remove(value);
                    LogInfo(fmt::format("Deselected id: {}", value));
                } else {
                    selection.add(value);
                    LogInfo(fmt::format("Selected id: {}", value));
                }

                brushingAndLinkingInport_.select(selection);

                LogInfo(fmt::format("Current selection: {}", selection));
            }

            p->markAsUsed();
        }

        invalidate(InvalidationLevel::Valid);
    }
}

void ContourExplorerProcessor::updateTF() {
    if (!inport_.hasData() || !inport_.getData()) return;

    const auto inputVolume = inport_.getData();

    const auto max = static_cast<int32_t>(inputVolume->dataMap_.dataRange.y);

    BitSet selection;

    selection.addRange(0, max + 1);

    const auto tfPrimitives =
        SegmentationTransferFunctionGenerator::generateTFPrimitivesForSegments(selection, max + 1);

    LogInfo(
        fmt::format("Generated {} tf primitives for {} segments.", tfPrimitives.size(), max + 1));

    NetworkLock l;

    transferFunction_.get().clear();

    transferFunction_.get().set(std::begin(tfPrimitives), std::end(tfPrimitives));
}

void ContourExplorerProcessor::generateIsoVolume() {
    const auto& selection = brushingAndLinkingInport_.getSelectedIndices();

    std::set s(std::begin(selection), std::end(selection));

    const auto inputVolume = inport_.getData();

    auto rawData = inputVolume->getRepresentation<VolumeRAM>()
                       ->dispatch<glm::u8*, dispatching::filter::Scalars>([&](auto vrprecision) {
                           const auto numberOfElements = glm::compMul(inputVolume->getDimensions());

                           auto rawInput = vrprecision->getDataTyped();
                           auto rawOutput = new glm::u8[numberOfElements];

                           std::fill_n(rawOutput, glm::compMul(inputVolume->getDimensions()), 0);

                           int i = 0;

                           std::transform(rawInput, rawInput + numberOfElements, rawOutput,
                                          [selection, &i](const auto val) {
                                              if (selection.contains(val)) {
                                                  i++;
                                                  return 1;
                                              }
                                              return 0;
                                          });

                           return rawOutput;
                       });

    const auto isoVolumeRamPrecision =
        std::make_shared<VolumeRAMPrecision<glm::u8>>(rawData, inputVolume->getDimensions());

    auto isoVolume = std::make_shared<Volume>(isoVolumeRamPrecision);

    isoVolume->setBasis(inputVolume->getBasis());
    isoVolume->setOffset(inputVolume->getOffset());
    isoVolume->setModelMatrix(inputVolume->getModelMatrix());
    isoVolume->setWorldMatrix(inputVolume->getWorldMatrix());

    isoVolume->dataMap_.dataRange = isoVolume->dataMap_.valueRange = dvec2{0, 1};

    isoVolumeOutport_.setData(isoVolume);
}

void ContourExplorerProcessor::process() {
    if (backgroundPort_.hasData() &&
        !shader_.getFragmentShaderObject()->hasShaderDefine("BACKGROUND_AVAILABLE")) {
        shader_.getFragmentShaderObject()->addShaderDefine("BACKGROUND_AVAILABLE");
        shader_.build();
    } else if (!backgroundPort_.hasData() &&
               shader_.getFragmentShaderObject()->hasShaderDefine("BACKGROUND_AVAILABLE")) {
        shader_.getFragmentShaderObject()->removeShaderDefine("BACKGROUND_AVAILABLE");
        shader_.build();
    }

    utilgl::activateTargetAndClearOrCopySource(outport_, backgroundPort_);
    shader_.activate();

    TextureUnitContainer units;
    utilgl::bindAndSetUniforms(shader_, units, inport_);
    utilgl::bindAndSetUniforms(shader_, units, transferFunction_);
    if (backgroundPort_.hasData()) {
        utilgl::bindAndSetUniforms(shader_, units, backgroundPort_, ImageType::ColorDepthPicking);
    }

    utilgl::setShaderUniforms(shader_, embeddedMesh_, "geometry");
    shader_.setUniform("pickColor", picking_.getColor(0));
    utilgl::setUniforms(shader_, camera_);

    MeshDrawerGL::DrawObject drawer(embeddedMesh_.getRepresentation<MeshGL>(),
                                    embeddedMesh_.getDefaultMeshInfo());
    drawer.draw();

    shader_.deactivate();
    utilgl::deactivateCurrentTarget();
}

}  // namespace inviwo
