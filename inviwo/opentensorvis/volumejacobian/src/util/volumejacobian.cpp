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

#include <inviwo/volumejacobian/util/volumejacobian.h>
#include <modules/eigenutils/eigenutils.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

#include <inviwo/core/util/volumeramutils.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/templatesampler.h>

namespace inviwo {

namespace volutil {

class EigenValueException : public Exception {
public:
    using Exception::Exception;
};

namespace detail {

template <typename F, typename T>
F componentMin(F a, const T &b) {
    return static_cast<F>(std::min(a, b));
}

template <typename F, glm::length_t L, typename T, glm::qualifier Q>
F componentMin(F a, const glm::vec<L, T, Q> &b) {
    for (glm::length_t i = 0; i < L; i++) {
        a = static_cast<F>(std::min(a, b[i]));
    }
    return a;
}

template <typename F, typename T>
F componentMax(F a, const T &b) {
    return static_cast<F>(std::max(a, b));
}

template <typename F, glm::length_t L, typename T, glm::qualifier Q>
F componentMax(F a, const glm::vec<L, T, Q> &b) {
    for (glm::length_t i = 0; i < L; i++) {
        a = static_cast<F>(std::max(a, b[i]));
    }
    return a;
}

template <typename Callback>
std::unique_ptr<Volume> volumeJacobian(const Volume &volume, Callback &&callback) {
    using ReturnType = std::invoke_result_t<Callback, mat3>;

    auto newVolumeRep = std::make_shared<VolumeRAMPrecision<ReturnType>>(volume.getDimensions());
    auto newVolume = std::make_unique<Volume>(newVolumeRep);

    newVolume->setModelMatrix(volume.getModelMatrix());
    newVolume->setWorldMatrix(volume.getWorldMatrix());
    newVolume->dataMap_ = volume.dataMap_;

    const auto m = newVolume->getCoordinateTransformer().getDataToWorldMatrix();

    const auto a = m * vec4(0, 0, 0, 1);
    const auto b = m * vec4(1.0f / vec3(volume.getDimensions() - size3_t(1)), 1);
    const auto spacing = b - a;

    const vec3 ox(spacing.x, 0, 0);
    const vec3 oy(0, spacing.y, 0);
    const vec3 oz(0, 0, spacing.z);

    volume.getRepresentation<VolumeRAM>()->dispatch<void, dispatching::filter::Vec3s>(
        [&](auto vol) {
            using ValueType = util::PrecisionValueType<decltype(vol)>;
            using ComponentType = typename ValueType::value_type;
            using FloatType =
                typename std::conditional_t<std::is_same_v<float, ComponentType>, float, double>;
            using Sampler = TemplateVolumeSampler<ValueType, FloatType>;

            util::IndexMapper3D index(volume.getDimensions());
            auto data = newVolumeRep->getDataTyped();
            float minV = std::numeric_limits<float>::max();
            float maxV = std::numeric_limits<float>::lowest();

            const auto worldSpace = Sampler::Space::World;
            const Sampler sampler(volume, worldSpace);

            util::forEachVoxelParallel(*vol, [&](const size3_t &pos) {
                const vec3 world{m * vec4((vec3(pos) + 0.5f) / vec3(volume.getDimensions()), 1)};

                const auto Fxp = static_cast<vec3>(sampler.sample(world + ox));
                const auto Fxm = static_cast<vec3>(sampler.sample(world - ox));
                const auto Fyp = static_cast<vec3>(sampler.sample(world + oy));
                const auto Fym = static_cast<vec3>(sampler.sample(world - oy));
                const auto Fzp = static_cast<vec3>(sampler.sample(world + oz));
                const auto Fzm = static_cast<vec3>(sampler.sample(world - oz));

                const mat3 J{(Fxp - Fxm) / (2.0f * spacing.x),  //
                             (Fyp - Fym) / (2.0f * spacing.y),  //
                             (Fzp - Fzm) / (2.0f * spacing.z)};

                const auto v = callback(J);

                minV = componentMin(minV, v);
                maxV = componentMax(maxV, v);

                data[index(pos)] = v;
            });

            if (minV >= 0) {
                newVolume->dataMap_.dataRange = newVolume->dataMap_.valueRange = dvec2(minV, maxV);
            } else {
                auto range = std::max(std::abs(minV), std::abs(maxV));
                newVolume->dataMap_.dataRange = dvec2(-range, range);
                newVolume->dataMap_.valueRange = dvec2(-range, range);
                // newVolume->dataMap_.valueRange = dvec2(minV, maxV);
            }
        });

    return newVolume;
}

}  // namespace detail

std::unique_ptr<Volume> curl(std::shared_ptr<const Volume> volume) { return curl(*volume); }

std::unique_ptr<Volume> curl(const Volume &volume) {
    return detail::volumeJacobian(volume, [](auto J) -> vec3 {
        return vec3{J[1].z - J[2].y, J[2].x - J[0].z, J[0].y - J[1].x};
    });
}

std::unique_ptr<Volume> eigenValues(std::shared_ptr<const Volume> volume) {
    return eigenValues(*volume);
}

std::unique_ptr<Volume> eigenValues(const Volume &volume) {
    return detail::volumeJacobian(volume, [](auto J) {
        if (glm::length2(J[0]) < 0.000001) {
            return vec3{0};
        }
        if (glm::length2(J[1]) < 0.000001) {
            return vec3{0};
        }
        if (glm::length2(J[2]) < 0.000001) {
            return vec3{0};
        }
        const auto JT = glm::transpose(J);
        const auto S = (J + JT) * 0.5f;
        const auto O = (J - JT) * 0.5f;
        auto M = util::glm2eigen(S * S + O * O);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(M);
        if (es.info() != Eigen::Success) {
            throw EigenValueException("Failed to compute eigen values",
                                      IvwContextCustom("volutil::eigenValues"));
        }

        const auto e = es.eigenvalues();
        vec3 ev{e(0), e(1), e(2)};

        auto sort = [](vec3 &v) {
            if (v.x < v.y) {
                std::swap(v.x, v.y);
            }
            if (v.y < v.z) {
                std::swap(v.y, v.z);
            }
            if (v.x < v.y) {
                std::swap(v.x, v.y);
            }
        };

        sort(ev);

        return ev;
        //return glm::clamp(ev, -1000.f, 1000.0f);
    });
}

std::unique_ptr<Volume> qValues(std::shared_ptr<const Volume> volume) { return qValues(*volume); }

std::unique_ptr<Volume> qValues(const Volume &volume) {
    return detail::volumeJacobian(volume, [](auto J) {
        const auto T = glm::matrixCompMult(J, glm::transpose(J));
        return -(glm::compAdd(T[0]) + glm::compAdd(T[1]) + glm::compAdd(T[2]))*0.5f;

        /*const auto S = (J + JT) * 0.5f;
        const auto O = (J - JT) * 0.5f;
        const auto M = S * S + O * O;*/
    });
}

std::unique_ptr<Volume> q2Values(std::shared_ptr<const Volume> volume) { return q2Values(*volume); }

std::unique_ptr<Volume> q2Values(const Volume &volume) {
    return detail::volumeJacobian(volume, [](auto J) {
        if (glm::length2(J[0]) < 0.000001) {
            return 0.0f;
        }
        if (glm::length2(J[1]) < 0.000001) {
            return 0.0f;
        }
        if (glm::length2(J[2]) < 0.000001) {
            return 0.0f;
        }
        const auto JT = glm::transpose(J);
        const auto S = (J + JT) * 0.5f;
        const auto O = (J - JT) * 0.5f;
        auto M = util::glm2eigen(S * S + O * O);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(M);
        if (es.info() != Eigen::Success) {
            throw EigenValueException("Failed to compute eigen values",
                                      IvwContextCustom("volutil::eigenValues"));
        }

        const auto e = es.eigenvalues();
        return -(e(0) + e(1) + e(2)) * 0.5f;
    });
}

std::unique_ptr<Volume> q3Values(std::shared_ptr<const Volume> volume) { return q3Values(*volume); }

std::unique_ptr<Volume> q3Values(const Volume &volume) {
    return detail::volumeJacobian(volume, [](auto J) {
        
        if (glm::length2(J[0]) < 0.000001) {
            return 0.0f;
        }
        if (glm::length2(J[1]) < 0.000001) {
            return 0.0f;
        }
        if (glm::length2(J[2]) < 0.000001) {
            return 0.0f;
        }
        const auto JT = glm::transpose(J);  
        const auto S = (J + JT) * 0.5f;
        const auto O = (J - JT) * 0.5f;

        float Snorm2 = 0;
        float Onorm2 = 0;

        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                Snorm2 += S[i][j] * S[i][j];
                Onorm2 += O[i][j] * O[i][j];
            }
        }
        return (Onorm2 - Snorm2) * 0.5f;
    });
}

}  // namespace volutil
}  // namespace inviwo
