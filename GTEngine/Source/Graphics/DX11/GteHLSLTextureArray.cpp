// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/09/12)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteHLSLTextureArray.h>
using namespace gte;


HLSLTextureArray::~HLSLTextureArray()
{
}

HLSLTextureArray::HLSLTextureArray(D3D_SHADER_INPUT_BIND_DESC const& desc)
    :
    HLSLResource(desc, 0)
{
    Initialize(desc);
}

HLSLTextureArray::HLSLTextureArray(D3D_SHADER_INPUT_BIND_DESC const& desc,
    unsigned int index)
    :
    HLSLResource(desc, index, 0)
{
    Initialize(desc);
}

void HLSLTextureArray::Initialize(D3D_SHADER_INPUT_BIND_DESC const& desc)
{
    mNumComponents = ((desc.uFlags >> 2) + 1);

    switch (desc.Dimension)
    {
    case D3D_SRV_DIMENSION_TEXTURE1DARRAY:
        mNumDimensions = 1;
        break;
    case D3D_SRV_DIMENSION_TEXTURE2DARRAY:
    case D3D_SRV_DIMENSION_TEXTURE2DMSARRAY:
    case D3D_SRV_DIMENSION_TEXTURECUBE:
    case D3D_SRV_DIMENSION_TEXTURECUBEARRAY:
        mNumDimensions = 2;
        break;
    default:
        mNumDimensions = 0;
        break;
    }

    mGpuWritable = (desc.Type == D3D_SIT_UAV_RWTYPED);
}

unsigned int HLSLTextureArray::GetNumComponents() const
{
    return mNumComponents;
}

unsigned int HLSLTextureArray::GetNumDimensions() const
{
    return mNumDimensions;
}

bool HLSLTextureArray::IsGpuWritable() const
{
    return mGpuWritable;
}

