#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"

layout (location = 0) in vec3 inColor;
layout (location = 1) in vec3 inPosition;
layout (location = 2) in vec3 inNormal;
layout (location = 3) in vec2 inTexCoord;

layout (location = 0) out vec4 outColor;
layout (location = 1) out vec4 outPosition;
layout (location = 2) out vec4 outNormal;

void main()
{
    uint seed = uint(gl_PrimitiveID);
    vec3 randColor = vec3(rand(seed), rand(seed), rand(seed));
    outColor = vec4(randColor, 1.0);
    outPosition = vec4(inPosition * 0.1, 1.0);
    outNormal = vec4(inNormal, 1.0);
}
