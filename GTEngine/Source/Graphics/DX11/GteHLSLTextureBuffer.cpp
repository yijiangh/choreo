// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/09/12)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteHLSLTextureBuffer.h>
using namespace gte;


HLSLTextureBuffer::~HLSLTextureBuffer()
{
}

HLSLTextureBuffer::HLSLTextureBuffer(
    D3D_SHADER_INPUT_BIND_DESC const& desc, unsigned int numBytes,
    std::vector<Member> const& members)
    :
    HLSLBaseBuffer(desc, numBytes, members)
{
}

HLSLTextureBuffer::HLSLTextureBuffer(
    D3D_SHADER_INPUT_BIND_DESC const& desc, unsigned int index,
    unsigned int numBytes, std::vector<Member> const& members)
    :
    HLSLBaseBuffer(desc, index, numBytes, members)
{
}

