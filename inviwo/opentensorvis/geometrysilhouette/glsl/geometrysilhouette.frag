/*********************************************************************************
*
* Inviwo - Interactive Visualization Workshop
*
* Copyright (c) 2014-2019 Inviwo Foundation
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
**********************************************************************************/

#include "utils/structs.glsl"

uniform bool overrideColor;
uniform vec4 color;
uniform float threshold;
uniform float opacity;
uniform bool binary;
uniform bool showBackfaces;
uniform float widenNarrow;
uniform int easeMethod;

in vec3 normal_;
in vec3 view_vector_;
in vec4 meshColor_;

const float PI = 3.14;

float easeIn(float val) {
    return 1.0 - cos(val * PI / 2.0);
}

float easeOut(float val) {
    return sin(val * PI / 2.0);
}

void main() {
    float cosAlpha = abs(dot(normal_, view_vector_) / (length(normal_) * length(view_vector_)));

    float val = 1.0 - cosAlpha;

    val = pow(val, widenNarrow);

    if (!binary) {
        if(overrideColor) {
            // hermite
    		if(easeMethod == 1) {
                FragData0 = vec4(color.r, color.g, color.b, smoothstep(0.0, 1.0, val * opacity));
            // ease in
    		} else if(easeMethod == 2) {
                FragData0 = vec4(color.r, color.g, color.b, easeIn(val * opacity));
            // ease out
            } else if(easeMethod == 3) {
                FragData0 = vec4(color.r, color.g, color.b, easeOut(val * opacity));
            // linear
            } else {
    			FragData0 = vec4(color.r, color.g, color.b, val * opacity);
    		}
        } else {
            // hermite
			if(easeMethod == 1) {
				FragData0 = vec4(meshColor_.r, meshColor_.g, meshColor_.b, smoothstep(0.0, 1.0, val * opacity));
            // ease in
			} else if(easeMethod == 2) {
                FragData0 = vec4(meshColor_.r, meshColor_.g, meshColor_.b, easeIn(val * opacity));
            // ease out
            } else if(easeMethod == 3) {
                FragData0 = vec4(meshColor_.r, meshColor_.g, meshColor_.b, easeOut(val * opacity));
            // linear
            } else {
				FragData0 = vec4(meshColor_.r, meshColor_.g, meshColor_.b, val * opacity);
			}
        }
    }
    if (binary && cosAlpha <= threshold) {
        if(overrideColor) {
            FragData0 = vec4(color.r, color.g, color.b, 1.0 * opacity);
        } else {
            FragData0 = vec4(meshColor_.r, meshColor_.g, meshColor_.b, 1.0 * opacity);
        }
    }
    if (binary && cosAlpha > threshold) {
        if (showBackfaces) {
            discard;
            return;
        }
        FragData0 = vec4(0.0);
    }
}
