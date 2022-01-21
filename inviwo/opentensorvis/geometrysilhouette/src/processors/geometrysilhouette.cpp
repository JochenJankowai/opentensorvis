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

#include <inviwo/geometrysilhouette/processors/geometrysilhouette.h>

#include <inviwo/core/rendering/meshdrawerfactory.h>
#include <inviwo/core/common/inviwoapplication.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo GeometrySilhouette::processorInfo_{
    "org.inviwo.GeometrySilhouette",  // Class identifier
    "Geometry Silhouette",            // Display name
    "Mesh Rendering",                 // Category
    CodeState::Stable,                // Code state
    Tags::GL,                         // Tags
};
const ProcessorInfo GeometrySilhouette::getProcessorInfo() const { return processorInfo_; }

GeometrySilhouette::GeometrySilhouette()
    : Processor()
    , inport_("inport")
    , image_("imageInport")
    , outport_("outport")
    , shader_("geometrysilhouette.vert", "geometrysilhouette.frag")
    , camera_("camera", "Camera")
    , tb_(&camera_)
    , overrideColor_("overrideColor", "Override mesh color", true)
    , colour_("colour", "Contour colour", vec4(vec3(0.0f), 1.0f), vec4(0.0f), vec4(1.0f),
              vec4(0.00001f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , binary_("binary", "Make contour binary", false)
    , threshold_("threshold", "Alpha threshold", 0.3f, 0.0f, 1.0f, 0.1f)
    , opacity_("opacity", "Opacity", 1.0f, 0.0f, 1.0f, 0.1f)
    , showBackfaces_("showBackfaces", "Show backfaces", false)
    , enableDepthTest_("enableDepthTest_", "Enable Depth Test", false)
    , widenNarrow_("widenNarrow", "Widen/Narrow gradient", 1.0f, 1.0f, 10.0f, 0.1f)
    , settings_("settings", "Contour settings")
    , easeMethod_("easeMethod", "Ease method",
                  {{"linear", "Linear", 0},
                   {"hermite", "Hermite", 1},
                   {"easeIn", "Ease in", 2},
                   {"easeOut", "Ease out", 3}}) {
    addPort(inport_);
    addPort(image_);
    addPort(outport_);

    image_.setOptional(true);

    settings_.addProperty(overrideColor_);
    settings_.addProperty(colour_);
    settings_.addProperty(widenNarrow_);
    settings_.addProperty(opacity_);
    settings_.addProperty(binary_);
    settings_.addProperty(threshold_);
    settings_.addProperty(showBackfaces_);
    settings_.addProperty(enableDepthTest_);
    settings_.addProperty(easeMethod_);

    addProperty(settings_);
    addProperty(camera_);
    addProperty(tb_);

    inport_.onChange([this]() { updateDrawers(); });

    overrideColor_.onChange([this]() { colour_.setVisible(overrideColor_.get()); });

    binary_.onChange([this]() { threshold_.setVisible(binary_.get()); });
}

void GeometrySilhouette::process() {
    if (image_.isReady()) {
        utilgl::activateTargetAndCopySource(outport_, image_);
    } else {
        utilgl::activateAndClearTarget(outport_);
    }

    shader_.activate();

    utilgl::setShaderUniforms(shader_, camera_, "camera_");
    shader_.setUniform("overrideColor", overrideColor_);
    shader_.setUniform("color", colour_);
    shader_.setUniform("threshold", threshold_);
    shader_.setUniform("opacity", opacity_);
    shader_.setUniform("binary", binary_);
    shader_.setUniform("showBackfaces", showBackfaces_);
    shader_.setUniform("widenNarrow", widenNarrow_);
    shader_.setUniform("easeMethod", easeMethod_.get());

    utilgl::CullFaceState cullFace(GL_FRONT);
    utilgl::GlBoolState depthTest(GL_DEPTH_TEST, enableDepthTest_.get());
    utilgl::BlendModeState blendModeStateGL(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (auto &drawer : drawers_) {
        utilgl::setShaderUniforms(shader_, *(drawer.second->getMesh()), "geometry_");
        drawer.second->draw();
    }

    if (showBackfaces_.get()) {
        utilgl::CullFaceState cullFace2(GL_BACK);
        for (auto &drawer : drawers_) {
            utilgl::setShaderUniforms(shader_, *(drawer.second->getMesh()), "geometry_");
            drawer.second->draw();
        }
    }

    shader_.deactivate();
    utilgl::deactivateCurrentTarget();
}

void GeometrySilhouette::updateDrawers() {
    auto changed = inport_.getChangedOutports();
    DrawerMap temp;
    std::swap(temp, drawers_);

    std::map<const Outport *, std::vector<std::shared_ptr<const Mesh>>> data;
    for (auto &elem : inport_.getSourceVectorData()) {
        data[elem.first].push_back(elem.second);
    }

    for (auto elem : data) {
        auto ibegin = temp.lower_bound(elem.first);
        auto iend = temp.upper_bound(elem.first);

        if (util::contains(changed, elem.first) || ibegin == temp.end() ||
            static_cast<long>(elem.second.size()) !=
                std::distance(ibegin, iend)) {  // data is changed or new.

            for (auto geo : elem.second) {
                if (auto renderer =
                        InviwoApplication::getPtr()->getMeshDrawerFactory()->create(geo.get())) {
                    drawers_.emplace(std::make_pair(elem.first, std::move(renderer)));
                }
            }
        } else {  // reuse the old data.
            drawers_.insert(std::make_move_iterator(ibegin), std::make_move_iterator(iend));
        }
    }
}

}  // namespace inviwo
