/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2020 Inviwo Foundation
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

#include <inviwo/core/ports/datainport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/datastructures/datatraits.h>
#include <fmt/format.h>

namespace inviwo {

/**
 * \ingroup ports
 */
using ExtremalPointsInport = DataInport<std::map<size_t, float>>;

/**
 * \ingroup ports
 */
using ExtremalPointsOutport = DataOutport<std::map<size_t, float>>;

template <>
struct DataTraits<std::map<size_t, float>> {
    static std::string classIdentifier() { return "org.inviwo.contourtree.ExtremalPoints"; }
    static std::string dataName() { return "ExtremalPoints"; }
    static uvec3 colorCode() { return uvec3(5, 255, 255); }
    static Document info(const std::map<size_t, float>&) {
        std::ostringstream oss;
        Document doc;
        doc.append("p", oss.str());
        return doc;
    }
};


}  // namespace inviwo

template <>
struct fmt::formatter<std::map<size_t, float>> {
    // Presentation format: 'f' - fixed, 'e' - exponential.
    char presentation = 'f';

    // Parses format specifications of the form ['f' | 'e'].
    constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin()) {
        // [ctx.begin(), ctx.end()) is a character range that contains a part of
        // the format string starting from the format specifications to be parsed,
        // e.g. in
        //
        //   fmt::format("{:f} - point of interest", point{1, 2});
        //
        // the range will contain "f} - point of interest". The formatter should
        // parse specifiers until '}' or the end of the range. In this example
        // the formatter should parse the 'f' specifier and return an iterator
        // pointing to '}'.

        auto it = ctx.begin(), end = ctx.end();
        // Check if reached the end of the range:
        if (it != end && *it != '}') throw format_error("invalid format");

        // Return an iterator past the end of the parsed range:
        return it;
    }

    // Formats the point p using the parsed format specification (presentation)
    // stored in this formatter.
    template <typename FormatContext>
    auto format(const std::map<size_t, float>& p, FormatContext& ctx) -> decltype(ctx.out()) {
        // ctx.out() is an output iterator to write to.

        for (const auto& item:p) {
            format_to(ctx.out(), "({}, {:.4f}),\n", item.first, item.second);
        }

        return ctx.out();
    }
};
