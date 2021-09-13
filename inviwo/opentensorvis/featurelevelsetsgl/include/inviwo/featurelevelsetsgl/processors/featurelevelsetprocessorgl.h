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

#include <inviwo/featurelevelsetsgl/featurelevelsetsglmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/ports/volumeport.h>
#include <modules/opengl/shader/shader.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/core/properties/propertyownerobserver.h>
#include <inviwo/opentensorviscompute/algorithm/volumereductiongl.h>
#include <inviwo/opentensorviscompute/algorithm/volumenormalizationgl.h>

namespace inviwo {

/** \docpage{org.inviwo.FeatureLevelSetProcessorGL, Feature Level Set Processor}
 * ![](org.inviwo.FeatureLevelSetProcessorGL.png?classIdentifier=org.inviwo.FeatureLevelSetProcessorGL)
 */
class IVW_MODULE_FEATURELEVELSETSGL_API FeatureLevelSetProcessorGL : public Processor,
                                                                     public PropertyOwnerObserver

{
public:
    FeatureLevelSetProcessorGL();
    virtual ~FeatureLevelSetProcessorGL() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void serialize(Serializer& s) const final;
    virtual void deserialize(Deserializer& s) final;

private:
    // Ports
    DataInport<Volume, 0, true> volumes_;
    VolumeOutport distanceVolumeOutport_;

    // Properties
    TemplateOptionProperty<GLuint> divisor_;
    BoolProperty squaredDistance_;
    BoolProperty useVolumesDataMap_;
    BoolProperty useNormalizedValues_;
    ListProperty traitPropertiesContainer_;

    // Caches
    size_t prevNumberOfVolumes_;
    GLuint prevDivisor_;
    std::array<vec2, 4> dataRangesCache_;
    size_t traitAllocation_{8};
    std::vector<std::string> volumeNameCache_{};
    double maxDist_;
    std::vector<std::pair<std::string, std::shared_ptr<Volume>>> normalizedVolumesCache_;

    // Members
    Shader shader_;
    ShaderSegment implicitFunctionSegment_;
    const size_t maxVolumes_{4};
    VolumeNormalizationGL normalization_;
    VolumeReductionGL reduction_;

    // Helpers
    std::vector<vec4> gatherPointTraits() const;
    void normalizePointTraits(std::vector<vec4>& pointTraits) const;

    /*
     * Gathers the point traits to upload to the GPU.
     * The layout is as follows:
     * Column defines the dimension in the attribute space
     * 1st row is the min value
     * 2nd row is the max value
     */
    std::vector<mat4> gatherRangeTraits() const;
    void normalizeRangeTraits(std::vector<mat4>& rangeTraits) const;

    // Code fragments
    void checkInput() const;
    void setUniforms();
    std::shared_ptr<Volume> bindOutputTexture();
    void gpuDispatch();

    // Updates
    void updateDataRangesCache();
    void updateMaximumDistance();
    void updateNormalizedVolumesCache();
    void updateVolumeNamesCache();

    /*
     * Returns true if input volumes have same identifiers as name cache
     */
    bool compareInputsToCache() const;

    void resizeTraitAllocation();
    void initializeAllProperties();

    virtual void onWillAddProperty(Property* property, size_t index) final;
};

}  // namespace inviwo
