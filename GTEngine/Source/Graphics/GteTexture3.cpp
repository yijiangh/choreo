// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTexture3.h>
using namespace gte;


Texture3::Texture3(DFType format, unsigned int width, unsigned int height,
    unsigned int thickness, bool hasMipmaps, bool createStorage)
    :
    TextureSingle(format, 3, width, height, thickness, hasMipmaps,
        createStorage)
{
    mType = GT_TEXTURE3;
}

unsigned int Texture3::GetWidth() const
{
    return TextureSingle::GetDimension(0);
}

unsigned int Texture3::GetHeight() const
{
    return TextureSingle::GetDimension(1);
}

unsigned int Texture3::GetThickness() const
{
    return TextureSingle::GetDimension(2);
}

