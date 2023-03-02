#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"

layout (location = 0) in VertexInput {
    vec4 pos;
    vec4 normal;
    vec4 texCoord;
    flat uint primID;
} vertexInput;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outPos;
layout(location = 2) out vec4 outNormal;

void main() {
    uint seed = vertexInput.primID;
    vec3 randColor = vec3(rand(seed), rand(seed), rand(seed));
    outColor = vec4(randColor, 1.0);

    outPos = vec4(vertexInput.pos.xyz * 0.1, 1);
    outNormal = vertexInput.normal;
}
