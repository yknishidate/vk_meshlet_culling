#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int64 : require
#extension GL_EXT_buffer_reference2 : require
#extension GL_EXT_nonuniform_qualifier : enable
layout(push_constant) uniform Constants {
    mat4 model;
    mat4 view;
    mat4 proj;
    uint64_t meshletBuffer;
    uint64_t meshletVertexBuffer;
    uint64_t meshletTriangleBuffer;
    uint64_t meshletBoundBuffer;
    int primitiveOffset;
    int meshletCount;
}
constants;

layout(binding = 1) buffer Vertices {
    float v[];
}
vertices;
layout(binding = 2) buffer Indices {
    uint i[];
}
indices;
layout(binding = 3) uniform Uniforms {
    mat4 prevView;
    mat4 prevProj;
    vec4 topFace;
    vec4 bottomFace;
    vec4 rightFace;
    vec4 leftFace;
    vec4 farFace;
    vec4 nearFace;
}
uniforms;

uint pcg(inout uint state) {
    uint prev = state * 747796405u + 2891336453u;
    uint word = ((prev >> ((prev >> 28u) + 4u)) ^ prev) * 277803737u;
    state = prev;
    return (word >> 22u) ^ word;
}

uvec2 pcg2d(uvec2 v) {
    v = v * 1664525u + 1013904223u;
    v.x += v.y * 1664525u;
    v.y += v.x * 1664525u;
    v = v ^ (v >> 16u);
    v.x += v.y * 1664525u;
    v.y += v.x * 1664525u;
    v = v ^ (v >> 16u);
    return v;
}

float rand(inout uint seed) {
    uint val = pcg(seed);
    return float(val) * (1.0 / float(0xffffffffu));
}
