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

        auto rest =
            colorbrewer::getColormap(colorbrewer::Family::Paired, 12);

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


TFPrimitiveSet SegmentationColorHelper::generateTFPrimitivesForSegments(
    const BitSet& selection, size_t numberOfSegments, double slope, const vec4& shadeColor) {

    auto colorMap = SegmentationColorHelper::getColorMapForNSegments(numberOfSegments);

    TFPrimitiveSet tfPoints;

    const auto dMaxValue = static_cast<double>(numberOfSegments) - 1.0;

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

}  // namespace inviwo