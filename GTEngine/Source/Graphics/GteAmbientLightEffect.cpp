// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteAmbientLightEffect.h>
using namespace gte;

AmbientLightEffect::AmbientLightEffect(std::shared_ptr<ProgramFactory> const& factory,
    BufferUpdater const& updater, std::shared_ptr<Material> const& material,
    std::shared_ptr<Lighting> const& lighting)
    :
    LightingEffect(factory, updater, msVSSource, msPSSource, material, lighting, nullptr)
{
    mMaterialConstant = std::make_shared<ConstantBuffer>(sizeof(InternalMaterial), true);
    UpdateMaterialConstant();
    mProgram->GetVShader()->Set("Material", mMaterialConstant);

    mLightingConstant = std::make_shared<ConstantBuffer>(sizeof(InternalLighting), true);
    UpdateLightingConstant();
    mProgram->GetVShader()->Set("Lighting", mLightingConstant);
}

void AmbientLightEffect::UpdateMaterialConstant()
{
    InternalMaterial* internalMaterial = mMaterialConstant->Get<InternalMaterial>();
    internalMaterial->emissive = mMaterial->emissive;
    internalMaterial->ambient = mMaterial->ambient;
    LightingEffect::UpdateMaterialConstant();
}

void AmbientLightEffect::UpdateLightingConstant()
{
    InternalLighting* internalLighting = mLightingConstant->Get<InternalLighting>();
    internalLighting->ambient = mLighting->ambient;
    internalLighting->attenuation = mLighting->attenuation;
    LightingEffect::UpdateLightingConstant();
}

std::string const AmbientLightEffect::msGLSLVSSource =
"uniform PVWMatrix\n"
"{\n"
"    mat4 pvwMatrix;\n"
"};\n"
"\n"
"uniform Material\n"
"{\n"
"    vec4 materialEmissive;\n"
"    vec4 materialAmbient;\n"
"};\n"
"\n"
"uniform Light\n"
"{\n"
"    vec4 lightingAmbient;\n"
"    vec4 lightingAttenuation;\n"
"};\n"
"\n"
"layout(location = 0) in vec3 modelPosition;\n"
"\n"
"layout(location = 0) out vec4 vertexColor;\n"
"\n"
"void main()\n"
"{\n"
"    vec3 ambient = lightingAttenuation.w * lightingAmbient.rgb;\n"
"    vertexColor.rgb = materialEmissive.rgb + materialAmbient.rgb * ambient;\n"
"    vertexColor.a = 1.0f;\n"
"#if GTE_USE_MAT_VEC\n"
"    gl_Position = pvwMatrix * vec4(modelPosition, 1.0f);\n"
"#else\n"
"    gl_Position = vec4(modelPosition, 1.0f) * pvwMatrix;\n"
"#endif\n"
"}\n";

std::string const AmbientLightEffect::msGLSLPSSource =
"layout(location = 0) in vec4 vertexColor;\n"
"\n"
"layout(location = 0) out vec4 pixelColor0;\n"
"\n"
"void main()\n"
"{\n"
"    pixelColor0 = vertexColor;\n"
"}\n";


std::string const AmbientLightEffect::msHLSLSource =
"cbuffer PVWMatrix\n"
"{\n"
"    float4x4 pvwMatrix;\n"
"};\n"
"\n"
"cbuffer Material\n"
"{\n"
"    float4 materialEmissive;\n"
"    float4 materialAmbient;\n"
"};\n"
"\n"
"cbuffer Light\n"
"{\n"
"    float4 lightingAmbient;\n"
"    float4 lightingAttenuation;\n"
"};\n"
"\n"
"struct VS_INPUT\n"
"{\n"
"    float3 modelPosition : POSITION;\n"
"};\n"
"\n"
"struct VS_OUTPUT\n"
"{\n"
"    float4 vertexColor : COLOR0;\n"
"    float4 clipPosition : SV_POSITION;\n"
"};\n"
"\n"
"VS_OUTPUT VSMain(VS_INPUT input)\n"
"{\n"
"    VS_OUTPUT output;\n"
"\n"
"    float3 ambient = lightingAttenuation.w * lightingAmbient.rgb;\n"
"    output.vertexColor.rgb = materialEmissive.rgb + materialAmbient.rgb * ambient;\n"
"    output.vertexColor.a = 1.0f;\n"
"#if GTE_USE_MAT_VEC\n"
"    output.clipPosition = mul(pvwMatrix, float4(input.modelPosition, 1.0f));\n"
"#else\n"
"    output.clipPosition = mul(float4(input.modelPosition, 1.0f), pvwMatrix);\n"
"#endif\n"
"    return output;\n"
"}\n"
"\n"
"struct PS_INPUT\n"
"{\n"
"    float4 vertexColor : COLOR0;\n"
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
"    output.pixelColor0 = input.vertexColor;\n"
"    return output;\n"
"}\n";

std::string const* AmbientLightEffect::msVSSource[] =
{
    &msGLSLVSSource,
    &msHLSLSource
};

std::string const* AmbientLightEffect::msPSSource[] =
{
    &msGLSLPSSource,
    &msHLSLSource
};
