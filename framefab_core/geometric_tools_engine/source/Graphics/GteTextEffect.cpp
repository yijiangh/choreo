// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Mathematics/GteVector2.h>
#include <Graphics/GteTextEffect.h>
using namespace gte;

TextEffect::TextEffect(std::shared_ptr<ProgramFactory> const& factory,
    std::shared_ptr<Texture2> const& texture)
{
    auto api = factory->GetAPI();
    mProgram = factory->CreateFromSources(*msVSSource[api], *msPSSource[api], "");
    if (mProgram)
    {
        mTranslate = std::make_shared<ConstantBuffer>(sizeof(Vector2<float>), true);
        mColor = std::make_shared<ConstantBuffer>(sizeof(Vector4<float>), true);
        mSamplerState = std::make_shared<SamplerState>();

        SetTranslate(0.0f, 0.0f);
        mProgram->GetVShader()->Set("Translate", mTranslate);

        SetColor({ 0.0f, 0.0f, 0.0f, 0.0f });
        mProgram->GetPShader()->Set("TextColor", mColor);
#if defined(GTE_DEV_OPENGL)
        mProgram->GetPShader()->Set("baseSampler", texture);
#else
        mProgram->GetPShader()->Set("baseTexture", texture);
#endif
        mProgram->GetPShader()->Set("baseSampler", mSamplerState);
    }
}

std::shared_ptr<ConstantBuffer> const& TextEffect::GetTranslate() const
{
    return mTranslate;
}

std::shared_ptr<ConstantBuffer> const& TextEffect::GetColor() const
{
    return mColor;
}

void TextEffect::SetTranslate(float x, float  y)
{
    float* data = mTranslate->Get<float>();
    data[0] = x;
    data[1] = y;
}

void TextEffect::SetColor(Vector4<float> const& color)
{
    Vector4<float>* data = mColor->Get<Vector4<float>>();
    *data = color;
}


std::string const TextEffect::msGLSLVSSource =
"uniform Translate\n"
"{\n"
"    vec2 translate;\n"
"};\n"
"\n"
"layout(location = 0) in vec2 modelPosition;\n"
"layout(location = 1) in vec2 modelTCoord;\n"
"layout(location = 0) out vec2 vertexTCoord;\n"
"\n"
"void main()\n"
"{\n"
"    vertexTCoord = modelTCoord;\n"
"    gl_Position.x = 2.0f * modelPosition.x - 1.0f + 2.0f * translate.x;\n"
"    gl_Position.y = 2.0f * modelPosition.y - 1.0f + 2.0f * translate.y;\n"
"    gl_Position.z = -1.0f;\n"
"    gl_Position.w = 1.0f;\n"
"}\n";

std::string const TextEffect::msGLSLPSSource =
"uniform TextColor\n"
"{\n"
"    vec4 textColor;\n"
"};\n"
"\n"
"layout(location = 0) in vec2 vertexTCoord;\n"
"layout(location = 0) out vec4 pixelColor;\n"
"\n"
"uniform sampler2D baseSampler;\n"
"\n"
"void main()\n"
"{\n"
"    float bitmapAlpha = texture(baseSampler, vertexTCoord).r;\n"
"    if (bitmapAlpha > 0.5f)\n"
"    {\n"
"        discard;\n"
"    }\n"
"    pixelColor = textColor;\n"
"}\n";

std::string const TextEffect::msHLSLSource =
"cbuffer Translate\n"
"{\n"
"    float2 translate;\n"
"};\n"
"struct VS_INPUT\n"
"{\n"
"    float2 modelPosition : POSITION;\n"
"    float2 modelTCoord : TEXCOORD0;\n"
"};\n"
"\n"
"struct VS_OUTPUT\n"
"{\n"
"    float2 vertexTCoord : TEXCOORD0;\n"
"    float4 clipPosition : SV_POSITION;\n"
"};\n"
"\n"
"VS_OUTPUT VSMain (VS_INPUT input)\n"
"{\n"
"    VS_OUTPUT output;\n"
"    output.vertexTCoord = input.modelTCoord;\n"
"    output.clipPosition.x = 2.0f*input.modelPosition.x - 1.0f + 2.0f*translate.x;\n"
"    output.clipPosition.y = 2.0f*input.modelPosition.y - 1.0f + 2.0f*translate.y;\n"
"    output.clipPosition.z = 0.0f;\n"
"    output.clipPosition.w = 1.0f;\n"
"    return output;\n"
"}\n"
"\n"
"cbuffer TextColor\n"
"{\n"
"    float4 textColor;\n"
"};\n"
"\n"
"Texture2D baseTexture;\n"
"SamplerState baseSampler;\n"
"\n"
"struct PS_INPUT\n"
"{\n"
"    float2 vertexTCoord : TEXCOORD0;\n"
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
"    float bitmapAlpha = baseTexture.Sample(baseSampler, input.vertexTCoord).r;\n"
"    if (bitmapAlpha > 0.5f)\n"
"    {\n"
"        discard;\n"
"    }\n"
"    output.pixelColor0 = textColor;\n"
"    return output;\n"
"}\n";

std::string const* TextEffect::msVSSource[] =
{
    &msGLSLVSSource,
    &msHLSLSource
};

std::string const* TextEffect::msPSSource[] =
{
    &msGLSLPSSource,
    &msHLSLSource
};
