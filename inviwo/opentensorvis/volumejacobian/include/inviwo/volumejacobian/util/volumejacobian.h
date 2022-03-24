/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018 Inviwo Foundation
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

#include <inviwo/volumejacobian/volumejacobianmoduledefine.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/opentensorvisbase/datastructures/tensorfield3d.h>

namespace inviwo {

namespace volutil {

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> curl(std::shared_ptr<const Volume> volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> curl(const Volume &volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> eigenValues(
    std::shared_ptr<const Volume> volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> eigenValues(const Volume &volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> qValues(std::shared_ptr<const Volume> volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> qValues(const Volume &volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> q2Values(std::shared_ptr<const Volume> volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> q2Values(const Volume &volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> q3Values(std::shared_ptr<const Volume> volume);

IVW_MODULE_VOLUMEJACOBIAN_API std::unique_ptr<Volume> q3Values(const Volume &volume);

enum class JacobianOperations { Curl, EigenValues, Q , Q2, Q3};
const static std::vector<JacobianOperations> AllJacobianOperations{
    JacobianOperations::Curl, JacobianOperations::EigenValues, JacobianOperations::Q,
    JacobianOperations::Q2, JacobianOperations::Q3};

template <class Elem, class Traits>
std::basic_ostream<Elem, Traits> &operator<<(std::basic_ostream<Elem, Traits> &os,
                                             JacobianOperations op) {
    switch (op) {
        case JacobianOperations::Curl:
            os << "Curl";
            break;
        case JacobianOperations::EigenValues:
            os << "EigenValues";
            break;
        case JacobianOperations::Q:
            os << "Q";
            break;
        case JacobianOperations::Q2:
            os << "Q2";
            break;
        case JacobianOperations::Q3:
            os << "Q3";
            break;
    }
    return os;
}

template <typename InVolume>
std::unique_ptr<Volume> jacobianOperation(const InVolume &v, JacobianOperations op) {
    switch (op) {
        case JacobianOperations::Curl:
            return curl(v);
        case JacobianOperations::EigenValues:
            return eigenValues(v);
        case JacobianOperations::Q:
            return qValues(v);
        case JacobianOperations::Q2:
            return q2Values(v);
        case JacobianOperations::Q3:
            return q3Values(v);
        default:
            throw Exception("Not yet implemented", IVW_CONTEXT_CUSTOM("volutil::jacobianOperation"));
    }
}

std::optional<std::shared_ptr<TensorField3D>> jacobian(std::shared_ptr<const Volume>);

}  // namespace volutil
}  // namespace inviwo
