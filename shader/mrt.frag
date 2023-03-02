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
    Materials materials = Materials(constants.materialBuffer);
    MaterialIDs materialIDs = MaterialIDs(constants.materialIDBuffer);
    Material material = materials.m[materialIDs.i[constants.primitiveOffset + gl_PrimitiveID]];

    vec4 diffuse = vec4(material.diffuse, 1);
    vec4 emission = vec4(material.emission, 1);
    if(material.diffuseTexture != -1){
        diffuse = texture(textures[material.diffuseTexture], inTexCoord);
    }
    if(material.emissionTexture != -1){
        emission = texture(textures[material.emissionTexture], inTexCoord);
    }

    vec3 screenPos = vec3(constants.proj * constants.view * vec4(inPosition, 1.0));
    vec3 prevScreenPos = vec3(uniforms.prevProj * uniforms.prevView * vec4(inPosition, 1.0));
    vec3 motionVector = screenPos - prevScreenPos;

    //uint seed = uint(materialIDs.i[gl_PrimitiveID]);
    //vec3 randColor = vec3(rand(seed), rand(seed), rand(seed));
    //outColor = vec4(randColor, 1.0);
    outColor = diffuse + emission;
    outPosition = vec4(inPosition * 0.1, 1.0);
    outNormal = vec4(inNormal, 1.0);
    //outPosition = vec4(abs(motionVector), 1.0);
}
