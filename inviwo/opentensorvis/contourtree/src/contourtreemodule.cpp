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

#include <inviwo/contourtree/contourtreemodule.h>
#include <inviwo/contourtree/processors/contourtreecomputationprocessor.h>
#include <inviwo/contourtree/processors/contourtreedataprocessor.h>
#include <inviwo/contourtree/processors/contourtreequeryprocessor.h>
#include <inviwo/contourtree/processors/contourtreesimplificationprocessor.h>
#include <inviwo/contourtree/ports/contourtreeport.h>
#include <inviwo/contourtree/ports/contourtreedataport.h>
#include <inviwo/contourtree/ports/contourtreesimplificationport.h>
#include <inviwo/contourtree/ports/contourtreetopologicalfeaturesport.h>
#include <inviwo/contourtree/processors/contourtreetopologicalfeaturesprocessor.h>
#include <inviwo/contourtree/processors/persistenceprocessor.h>
#include <inviwo/contourtree/processors/persistencesimplificationprocessor.h>

namespace inviwo {

ContourTreeModule::ContourTreeModule(InviwoApplication* app) : InviwoModule(app, "ContourTree") {
    registerPort<ContourTreeInport>();
    registerPort<ContourTreeOutport>();
    registerPort<ContourTreeDataInport>();
    registerPort<ContourTreeDataOutport>();
    registerPort<ContourTreeSimplificationInport>();
    registerPort<ContourTreeSimplificationOutport>();
    registerPort<ContourTreeTopologicalFeaturesInport>();
    registerPort<ContourTreeTopologicalFeaturesOutport>();

    registerProcessor<ContourTreeComputationProcessor>();
    registerProcessor<ContourTreeDataProcessor>();
    registerProcessor<ContourTreeQueryProcessor>();
    registerProcessor<ContourTreeSimplificationProcessor>();
    registerProcessor<ContourTreeTopologicalFeaturesProcessor>();
    registerProcessor<PersistenceProcessor>();
    registerProcessor<PersistenceSimplificationProcessor>();
}

}  // namespace inviwo
