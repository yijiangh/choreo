// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/11/13)

#include <GTEnginePCH.h>
#include <Graphics/GteConstantColorEffect.h>
using namespace gte;

ConstantColorEffect::ConstantColorEffect(std::shared_ptr<ProgramFactory> const& factory,
    Vector4<float> const& color)
    :
    mPVWMatrix(nullptr),
    mColor(nullptr)
{
    int i = factory->GetAPI();
    mProgram = factory->CreateFromSources(*msVSSource[i], *msPSSource[i], "");
    if (mProgram)
    {
        mPVWMatrixConstant = std::make_shared<ConstantBuffer>(sizeof(Matrix4x4<float>), true);
        mPVWMatrix = mPVWMatrixConstant->Get<Matrix4x4<float>>();
        *mPVWMatrix = Matrix4x4<float>::Identity();

        mColorConstant = std::make_shared<ConstantBuffer>(sizeof(Vector4<float>), true);
        mColor = mColorConstant->Get<Vector4<float>>();
        *mColor = color;

        mProgram->GetVShader()->Set("PVWMatrix", mPVWMatrixConstant);
        mProgram->GetVShader()->Set("ConstantColor", mColorConstant);
    }
}

void ConstantColorEffect::SetPVWMatrixConstant(std::shared_ptr<ConstantBuffer> const& pvwMatrix)
{
    mPVWMatrixConstant = pvwMatrix;
    mProgram->GetVShader()->Set("PVWMatrix", mPVWMatrixConstant);
}


std::string const ConstantColorEffect::msGLSLVSSource =
"uniform PVWMatrix\n"
"{\n"
"    mat4 pvwMatrix;\n"
"};\n"
"\n"
"uniform ConstantColor\n"
"{\n"
"    vec4 constantColor;\n"
"};\n"
"\n"
"layout(location = 0) in vec3 modelPosition;\n"
"\n"
"layout(location = 0) out vec4 vertexColor;\n"
"\n"
"void main()\n"
"{\n"
"#if GTE_USE_MAT_VEC\n"
"    gl_Position = pvwMatrix * vec4(modelPosition, 1.0f);\n"
"#else\n"
"    gl_Position = vec4(modelPosition, 1.0f) * pvwMatrix;\n"
"#endif\n"
"    vertexColor = constantColor;\n"
"}\n";

std::string const ConstantColorEffect::msGLSLPSSource =
"layout(location = 0) in vec4 vertexColor;\n"
"\n"
"layout(location = 0) out vec4 pixelColor0;\n"
"\n"
"void main()\n"
"{\n"
"    pixelColor0 = vertexColor;\n"
"}\n";

std::string const ConstantColorEffect::msHLSLSource =
"cbuffer PVWMatrix\n"
"{\n"
"    float4x4 pvwMatrix;\n"
"};\n"
"\n"
"cbuffer ConstantColor\n"
"{\n"
"    float4 constantColor;\n"
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
"#if GTE_USE_MAT_VEC\n"
"    output.clipPosition = mul(pvwMatrix, float4(input.modelPosition, 1.0f));\n"
"#else\n"
"    output.clipPosition = mul(float4(input.modelPosition, 1.0f), pvwMatrix);\n"
"#endif\n"
"    output.vertexColor = constantColor;\n"
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

std::string const* ConstantColorEffect::msVSSource[] =
{
    &msGLSLVSSource,
    &msHLSLSource
};

std::string const* ConstantColorEffect::msPSSource[] =
{
    &msGLSLPSSource,
    &msHLSLSource
};
