// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GtePointLightTextureEffect.h>
using namespace gte;

PointLightTextureEffect::PointLightTextureEffect(std::shared_ptr<ProgramFactory> const& factory,
    BufferUpdater const& updater, std::shared_ptr<Material> const& material,
    std::shared_ptr<Lighting> const& lighting, std::shared_ptr<LightCameraGeometry> const& geometry,
    std::shared_ptr<Texture2> const& texture, SamplerState::Filter filter, SamplerState::Mode mode0,
    SamplerState::Mode mode1)
    :
    LightingEffect(factory, updater, msVSSource, msPSSource, material, lighting, geometry),
    mTexture(texture)
{
    mSampler = std::make_shared<SamplerState>();
    mSampler->filter = filter;
    mSampler->mode[0] = mode0;
    mSampler->mode[1] = mode1;

    mMaterialConstant = std::make_shared<ConstantBuffer>(sizeof(InternalMaterial), true);
    UpdateMaterialConstant();

    mLightingConstant = std::make_shared<ConstantBuffer>(sizeof(InternalLighting), true);
    UpdateLightingConstant();

    mGeometryConstant = std::make_shared<ConstantBuffer>(sizeof(InternalGeometry), true);
    UpdateGeometryConstant();

    mProgram->GetPShader()->Set("Material", mMaterialConstant);
    mProgram->GetPShader()->Set("Lighting", mLightingConstant);
    mProgram->GetPShader()->Set("LightCameraGeometry", mGeometryConstant);
#if defined(GTE_DEV_OPENGL)
    mProgram->GetPShader()->Set("baseSampler", mTexture);
#else
    mProgram->GetPShader()->Set("baseTexture", mTexture);
#endif
    mProgram->GetPShader()->Set("baseSampler", mSampler);
}

void PointLightTextureEffect::UpdateMaterialConstant()
{
    InternalMaterial* internalMaterial = mMaterialConstant->Get<InternalMaterial>();
    internalMaterial->emissive = mMaterial->emissive;
    internalMaterial->ambient = mMaterial->ambient;
    internalMaterial->diffuse = mMaterial->diffuse;
    internalMaterial->specular = mMaterial->specular;
    LightingEffect::UpdateMaterialConstant();
}

void PointLightTextureEffect::UpdateLightingConstant()
{
    InternalLighting* internalLighting = mLightingConstant->Get<InternalLighting>();
    internalLighting->ambient = mLighting->ambient;
    internalLighting->diffuse = mLighting->diffuse;
    internalLighting->specular = mLighting->specular;
    internalLighting->attenuation = mLighting->attenuation;
    LightingEffect::UpdateLightingConstant();
}

void PointLightTextureEffect::UpdateGeometryConstant()
{
    InternalGeometry* internalGeometry = mGeometryConstant->Get<InternalGeometry>();
    internalGeometry->lightModelPosition = mGeometry->lightModelPosition;
    internalGeometry->cameraModelPosition = mGeometry->cameraModelPosition;
    LightingEffect::UpdateGeometryConstant();
}

std::string const PointLightTextureEffect::msGLSLVSSource =
"uniform PVWMatrix\n"
"{\n"
"    mat4 pvwMatrix;\n"
"};\n"
"\n"
"layout(location = 0) in vec3 modelPosition;\n"
"layout(location = 1) in vec3 modelNormal;\n"
"layout(location = 2) in vec2 modelTCoord;\n"
"\n"
"layout(location = 0) out vec3 vertexPosition;\n"
"layout(location = 1) out vec3 vertexNormal;\n"
"layout(location = 2) out vec2 vertexTCoord;\n"
"\n"
"void main()\n"
"{\n"
"    vertexPosition = modelPosition;\n"
"    vertexNormal = modelNormal;\n"
"    vertexTCoord = modelTCoord;\n"
"#if GTE_USE_MAT_VEC\n"
"    gl_Position = pvwMatrix * vec4(modelPosition, 1.0f);\n"
"#else\n"
"    gl_Position = vec4(modelPosition, 1.0f) * pvwMatrix;\n"
"#endif\n"
"}\n";

std::string const PointLightTextureEffect::msGLSLPSSource =
GetShaderSourceLitFunctionGLSL() +
"uniform Material\n"
"{\n"
"    vec4 materialEmissive;\n"
"    vec4 materialAmbient;\n"
"    vec4 materialDiffuse;\n"
"    vec4 materialSpecular;\n"
"};\n"
"\n"
"uniform Lighting\n"
"{\n"
"    vec4 lightingAmbient;\n"
"    vec4 lightingDiffuse;\n"
"    vec4 lightingSpecular;\n"
"    vec4 lightingAttenuation;\n"
"};\n"
"\n"
"uniform LightCameraGeometry\n"
"{\n"
"    vec4 lightModelPosition;\n"
"    vec4 cameraModelPosition;\n"
"};\n"
"\n"
"uniform sampler2D baseSampler;\n"
"\n"
"layout(location = 0) in vec3 vertexPosition;\n"
"layout(location = 1) in vec3 vertexNormal;\n"
"layout(location = 2) in vec2 vertexTCoord;\n"
"\n"
"layout(location = 0) out vec4 pixelColor0;\n"
"\n"
"void main()\n"
"{\n"
"    vec3 modelLightDiff = vertexPosition - lightModelPosition.xyz;\n"
"    vec3 vertexDirection = normalize(modelLightDiff);\n"
"    float NDotL = -dot(vertexNormal, vertexDirection);\n"
"    vec3 viewVector = normalize(cameraModelPosition.xyz - vertexPosition.xyz);\n"
"    vec3 halfVector = normalize(viewVector - vertexDirection);\n"
"    float NDotH = dot(vertexNormal, halfVector);\n"
"    vec4 lighting = lit(NDotL, NDotH, materialSpecular.a);\n"
"    vec3 lightingColor = materialAmbient.rgb * lightingAmbient.rgb +\n"
"        lighting.y * materialDiffuse.rgb * lightingDiffuse.rgb +\n"
"        lighting.z * materialSpecular.rgb * lightingSpecular.rgb;\n"
"\n"
"    float distance = length(modelLightDiff);\n"
"    float attenuation = lightingAttenuation.w / (lightingAttenuation.x + distance *\n"
"        (lightingAttenuation.y + distance * lightingAttenuation.z));\n"
"\n"
"    vec4 textureColor = texture(baseSampler, vertexTCoord);\n"
"\n"
"    vec3 color = lightingColor * textureColor.rgb;\n"
"    pixelColor0.rgb = materialEmissive.rgb + attenuation * color;\n"
"    pixelColor0.a = materialDiffuse.a * textureColor.a;\n"
"}\n";


std::string const PointLightTextureEffect::msHLSLSource =
"cbuffer PVWMatrix\n"
"{\n"
"    float4x4 pvwMatrix;\n"
"};\n"
"\n"
"struct VS_INPUT\n"
"{\n"
"    float3 modelPosition : POSITION;\n"
"    float3 modelNormal : NORMAL;\n"
"    float2 modelTCoord : TEXCOORD0;\n"
"};\n"
"\n"
"struct VS_OUTPUT\n"
"{\n"
"    float3 vertexPosition : TEXCOORD0;\n"
"    float3 vertexNormal : TEXCOORD1;\n"
"    float2 vertexTCoord : TEXCOORD2;\n"
"    float4 clipPosition : SV_POSITION;\n"
"};\n"
"\n"
"VS_OUTPUT VSMain(VS_INPUT input)\n"
"{\n"
"    VS_OUTPUT output;\n"
"\n"
"    output.vertexPosition = input.modelPosition;\n"
"    output.vertexNormal = input.modelNormal;\n"
"    output.vertexTCoord = input.modelTCoord;\n"
"#if GTE_USE_MAT_VEC\n"
"    output.clipPosition = mul(pvwMatrix, float4(input.modelPosition, 1.0f));\n"
"#else\n"
"    output.clipPosition = mul(float4(input.modelPosition, 1.0f), pvwMatrix);\n"
"#endif\n"
"    return output;\n"
"}\n"
"\n"
"cbuffer Material\n"
"{\n"
"    float4 materialEmissive;\n"
"    float4 materialAmbient;\n"
"    float4 materialDiffuse;\n"
"    float4 materialSpecular;\n"
"};\n"
"\n"
"cbuffer Lighting\n"
"{\n"
"    float4 lightingAmbient;\n"
"    float4 lightingDiffuse;\n"
"    float4 lightingSpecular;\n"
"    float4 lightingAttenuation;\n"
"};\n"
"\n"
"cbuffer LightCameraGeometry\n"
"{\n"
"    float4 lightModelPosition;\n"
"    float4 cameraModelPosition;\n"
"};\n"
"\n"
"Texture2D<float4> baseTexture;\n"
"SamplerState baseSampler;\n"
"\n"
"struct PS_INPUT\n"
"{\n"
"    float3 vertexPosition : TEXCOORD0;\n"
"    float3 vertexNormal : TEXCOORD1;\n"
"    float2 vertexTCoord : TEXCOORD2;\n"
"};\n"
"\n"
"struct PS_OUTPUT\n"
"{\n"
"    float4 pixelColor0 : SV_TARGET0;\n"
"};\n"
"\n"
"PS_OUTPUT PSMain(PS_INPUT input)\n"
"{\n"
"    PS_OUTPUT output;\n"
"\n"
"    float3 modelLightDiff = input.vertexPosition - lightModelPosition.xyz;\n"
"    float3 vertexDirection = normalize(modelLightDiff);\n"
"    float NDotL = -dot(input.vertexNormal, vertexDirection);\n"
"    float3 viewVector = normalize(cameraModelPosition.xyz - input.vertexPosition.xyz);\n"
"    float3 halfVector = normalize(viewVector - vertexDirection);\n"
"    float NDotH = dot(input.vertexNormal, halfVector);\n"
"    float4 lighting = lit(NDotL, NDotH, materialSpecular.a);\n"
"    float3 lightingColor = materialAmbient.rgb * lightingAmbient.rgb +\n"
"        lighting.y * materialDiffuse.rgb * lightingDiffuse.rgb +\n"
"        lighting.z * materialSpecular.rgb * lightingSpecular.rgb;\n"
"\n"
"    float distance = length(modelLightDiff);\n"
"    float attenuation = lightingAttenuation.w / (lightingAttenuation.x + distance *\n"
"        (lightingAttenuation.y + distance * lightingAttenuation.z));\n"
"\n"
"    float4 textureColor = baseTexture.Sample(baseSampler, input.vertexTCoord);\n"
"\n"
"    float3 color = lightingColor * textureColor.rgb;\n"
"    output.pixelColor0.rgb = materialEmissive.rgb + attenuation * color;\n"
"    output.pixelColor0.a = materialDiffuse.a * textureColor.a;\n"
"    return output;\n"
"}\n";

std::string const* PointLightTextureEffect::msVSSource[] =
{
    &msGLSLVSSource,
    &msHLSLSource
};

std::string const* PointLightTextureEffect::msPSSource[] =
{
    &msGLSLPSSource,
    &msHLSLSource
};
