// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/11/13)

#include <GTEnginePCH.h>
#include <Graphics/GteLightingEffect.h>
#include <Mathematics/GteMatrix4x4.h>
using namespace gte;

LightingEffect::LightingEffect(std::shared_ptr<ProgramFactory> const& factory,
    BufferUpdater const& updater, std::string const* vsSource[], std::string const* psSource[],
    std::shared_ptr<Material> const& material, std::shared_ptr<Lighting> const& lighting,
    std::shared_ptr<LightCameraGeometry> const& geometry)
    :
    mMaterial(material),
    mLighting(lighting),
    mGeometry(geometry)
{
    int api = factory->GetAPI();
    mProgram = factory->CreateFromSources(*vsSource[api], *psSource[api], "");
    if (mProgram)
    {
        mBufferUpdater = updater;
        mPVWMatrixConstant = std::make_shared<ConstantBuffer>(sizeof(Matrix4x4<float>), true);
        mProgram->GetVShader()->Set("PVWMatrix", mPVWMatrixConstant);
    }
}

void LightingEffect::SetPVWMatrixConstant(std::shared_ptr<ConstantBuffer> const& pvwMatrix)
{
    mPVWMatrixConstant = pvwMatrix;
    mProgram->GetVShader()->Set("PVWMatrix", mPVWMatrixConstant);
}

void LightingEffect::UpdateMaterialConstant()
{
    if (mMaterialConstant)
    {
        mBufferUpdater(mMaterialConstant);
    }
}

void LightingEffect::UpdateLightingConstant()
{
    if (mLightingConstant)
    {
        mBufferUpdater(mLightingConstant);
    }
}

void LightingEffect::UpdateGeometryConstant()
{
    if (mGeometryConstant)
    {
        mBufferUpdater(mGeometryConstant);
    }
}

std::string LightingEffect::GetShaderSourceLitFunctionGLSL()
{
    // HLSL and Cg have a 'lit' function for computing coefficients of the
    // ambient, diffuse, and specular lighting contributions.  GLSL does not.
    // This string is prepended to any GLSL shader that needs the 'lit'
    // function.
    //  ambient = 1.
    //  diffuse = ((n dot l) < 0) ? 0 : n dot l.
    //  specular = ((n dot l) < 0) || ((n dot h) < 0) ? 0 : (pow(n dot h, m)).
    // Where the vector N is the normal vector, L is the direction to light
    // and H is the half vector, and all three vectors are unit length.  The
    // inputs are NdotL = Dot(N,L), NdotH = Dot(N,H), and m is the specular
    // exponent (stored in Material:diffuse[3] in GTEngine).

    static std::string const sourceGLSL =
    "vec4 lit(float NdotL, float NdotH, float m)\n"
    "{\n"
    "  float ambient = 1.0;\n"
    "  float diffuse = max(NdotL, 0.0);\n"
    "  float specular = step(0.0, NdotL) * pow(max(NdotH, 0.0), m);\n"
    "  return vec4(ambient, diffuse, specular, 1.0);\n"
    "}\n";

    return sourceGLSL;
}

