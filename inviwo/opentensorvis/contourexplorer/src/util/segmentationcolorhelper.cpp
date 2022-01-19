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

#include <inviwo/contourexplorer/util/segmentationcolorhelper.h>

namespace inviwo {
std::vector<dvec4> SegmentationColorHelper::getColorMapForNSegments(size_t numberOfSegments) {
    std::vector<dvec4> colorMap;
    if (numberOfSegments > 24) {
        for (size_t i{0}; i < numberOfSegments; ++i) {
            colorMap.emplace_back(
                dvec3(static_cast<double>(i + 1) / static_cast<double>(numberOfSegments)), 1.0);
        }
    } else if (numberOfSegments > 12) {
        colorMap = colorbrewer::getColormap(colorbrewer::Family::Set3, 12);

        auto rest = colorbrewer::getColormap(colorbrewer::Family::Paired, 12);

        rest.resize(numberOfSegments - 12);

        colorMap.insert(std::begin(colorMap), std::begin(rest), std::end(rest));
    } else if (numberOfSegments < 3) {
        colorMap.emplace_back(0.2);
        if (numberOfSegments == 2) colorMap.emplace_back(0.8);
    } else {
        colorMap = colorbrewer::getColormap(colorbrewer::Family::Set3, numberOfSegments);
    }

    return colorMap;
}

TFPrimitiveSet SegmentationColorHelper::generateTFPrimitivesForSegments(const BitSet& selection,
                                                                        size_t numberOfSegments,
                                                                        double slope,
                                                                        const vec4& shadeColor) {

    const auto colorMap = SegmentationColorHelper::getColorMapForNSegments(numberOfSegments);

    TFPrimitiveSet tfPoints;

    const auto dMaxValue = static_cast<double>(numberOfSegments) - 1.0;

    if (selection.size() == 1) {
        tfPoints.add(0.5, colorMap[0]);
        return tfPoints;
    }

    for (auto id : selection) {
        // We'll handle these later
        if (id == 0 || id == numberOfSegments - 1) continue;

        const auto p0 = static_cast<double>(id - 1) + 0.5 + std::numeric_limits<double>::epsilon();
        const auto p3 =
            static_cast<double>(id - 1) + 0.5 + 1.0 - std::numeric_limits<double>::epsilon();

        const auto offset = (p3 - p0) * slope;
        const auto p1 = p0 + offset;
        const auto p2 = p3 - offset;

        tfPoints.add(p0 / dMaxValue, shadeColor);
        tfPoints.add(p1 / dMaxValue, colorMap.at(id));
        tfPoints.add(p2 / dMaxValue, colorMap.at(id));
        tfPoints.add(p3 / dMaxValue, shadeColor);
    }

    if (selection.contains(0)) {
        constexpr auto p1 = 0.5 - std::numeric_limits<double>::epsilon();
        constexpr auto p0 = p1 - std::numeric_limits<double>::epsilon();

        tfPoints.add(p0 / dMaxValue, colorMap[0]);
        tfPoints.add(p1 / dMaxValue, shadeColor);
    }

    if (selection.contains(numberOfSegments - 1)) {
        const auto p0 = dMaxValue - 0.5 + std::numeric_limits<double>::epsilon();
        const auto p1 = p0 + std::numeric_limits<double>::epsilon();

        tfPoints.add(p0 / dMaxValue, shadeColor);
        tfPoints.add(p1 / dMaxValue, colorMap[numberOfSegments - 1]);
    }

    return tfPoints;
}

dvec3 SegmentationColorHelper::hclToRgb(const dvec3& hcl) {
    {
        dvec3 rgb{};

        if (hcl.z != 0.0) {
            constexpr auto pi = 3.1415926536;
            constexpr auto hclGamma = 3.0;
            constexpr auto hclY0 = 100.0;
            constexpr auto hclMaxL = 0.530454533953517;

            auto h = hcl.x;
            const auto c = hcl.y;
            const auto l = hcl.z * hclMaxL;
            const auto q = glm::exp((1.0 - c / (2.0 * l)) * (hclGamma / hclY0));
            const auto u = (2.0 * l - c) / (2.0 * q - 1.0);
            const auto v = c / q;
            const auto t = glm::tan(
                (h + glm::min(glm::fract(2.0 * h) / 4.0, glm::fract(-2.0 * h) / 8.0)) * pi * 2.0);
            h *= 6.0;
            if (h <= 1.0) {
                rgb.r = 1.0;
                rgb.g = t / (1.0 + t);
            } else if (h <= 2.0) {
                rgb.r = (1.0 + t) / t;
                rgb.g = 1.0;
            } else if (h <= 3.0) {
                rgb.g = 1.0;
                rgb.b = 1.0 + t;
            } else if (h <= 4.0) {
                rgb.g = 1.0 / (1.0 + t);
                rgb.b = 1.0;
            } else if (h <= 5.0) {
                rgb.r = -1.0 / t;
                rgb.b = 1.0;
            } else {
                rgb.r = 1.0;
                rgb.b = -t;
            }
            rgb = rgb * v + u;
        }

        return rgb;
    }
}

dvec3 SegmentationColorHelper::rgbToHcl(const dvec3& rgb) {
    {
        constexpr auto hclGamma = 3.0;
        constexpr auto hclY0 = 100.0;
        constexpr auto hclMaxL = 0.530454533953517;

        dvec3 hcl{};
        auto h = 0.0;
        const auto u = glm::min(rgb.r, glm::min(rgb.g, rgb.b));
        const auto v = glm::max(rgb.r, glm::max(rgb.g, rgb.b));
        auto q = hclGamma / hclY0;
        hcl.y = v - u;

        if (hcl.y != 0.0) {
            constexpr auto pi = 3.1415926536;
            h = glm::atan(rgb.g - rgb.b, rgb.r - rgb.g) / pi;
            q *= u / v;
        }

        q = exp(q);
        hcl.x = glm::fract(h / 2.0 - glm::min(glm::fract(h), glm::fract(-h)) / 6.0);
        hcl.y *= q;
        hcl.z = glm::mix(-u, v, q) / (hclMaxL * 2.0);

        return hcl;
    }
}

}  // namespace inviwo