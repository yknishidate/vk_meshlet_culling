#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inTexCoord;

layout (location = 0) out vec3 outColor;
layout (location = 1) out vec3 outPosition;
layout (location = 2) out vec3 outNormal;
layout (location = 3) out vec2 outTexCoord;

void main() 
{
    gl_Position = constants.proj * constants.view * constants.model * vec4(inPosition, 1.0);
    outPosition = vec3(constants.model * vec4(inPosition, 1.0));
    outNormal = normalize(inNormal);	
    outColor = vec3(1.0);
    outTexCoord = inTexCoord;
}
