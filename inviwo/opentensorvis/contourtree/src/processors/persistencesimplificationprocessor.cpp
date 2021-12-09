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

#include <inviwo/contourtree/processors/persistencesimplificationprocessor.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/contourtree/util/util.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo PersistenceSimplificationProcessor::processorInfo_{
    "org.inviwo.PersistenceSimplificationProcessor",  // Class identifier
    "Persistence Simplification Processor",           // Display name
    "Undefined",                                      // Category
    CodeState::Experimental,                          // Code state
    Tags::None,                                       // Tags
};
const ProcessorInfo PersistenceSimplificationProcessor::getProcessorInfo() const {
    return processorInfo_;
}

PersistenceSimplificationProcessor::PersistenceSimplificationProcessor()
    : Processor()
    , contourTreeDataInport_("contourTreeDataInport")
    , contourTreeSimplificationOutport_("contourTreeSimplificationOutport")
    , persistence_("persistence", "Persistence")
    , persistenceObject_(nullptr) {

    addPorts(contourTreeDataInport_, contourTreeSimplificationOutport_);
    addProperties(persistence_);

    contourTreeDataInport_.onChange([this]() {
        if (!util::checkPorts(contourTreeDataInport_)) return;

        NetworkLock l;

        persistenceObject_ =
            std::make_shared<contourtree::Persistence>(contourTreeDataInport_.getData());

        simplifyCt_ = std::make_shared<contourtree::SimplifyCT>();

        simplifyCt_->setInput(contourTreeDataInport_.getData());
        simplifyCt_->initSimplification(persistenceObject_);

        const auto min = persistenceObject_->getMinPersistence();
        const auto max = persistenceObject_->getMaxPersistence();

        if (persistence_.get() < min && persistence_.get() > max) {
            persistence_.set(min);
        }

        persistence_.setMinValue(min);
        persistence_.setMaxValue(max);
    });
}

void PersistenceSimplificationProcessor::process() {
    if (!util::checkPorts(contourTreeDataInport_)) return;

    simplifyCt_->queue =
        std::priority_queue<uint32_t, std::vector<uint32_t>, contourtree::BranchCompare>{};

    simplifyCt_->simplify(persistence_.get());

    contourTreeSimplificationOutport_.setData(
        std::make_shared<contourtree::SimplifyCT>(simplifyCt_));
}

}  // namespace inviwo
