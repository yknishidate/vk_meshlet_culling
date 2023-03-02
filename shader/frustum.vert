#version 460
layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inTexCoord;

layout(push_constant) uniform Constants {
    mat4 view;
    mat4 proj;
    mat4 invView;
    mat4 invProj;
} constants;

void main() {
    vec4 worldPos = constants.invView * constants.invProj * vec4(inPosition, 1.0);
    gl_Position = constants.proj * constants.view * worldPos;
}
