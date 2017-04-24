// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTexture2Array.h>
using namespace gte;


Texture2Array::Texture2Array(unsigned int numItems, DFType format,
    unsigned int width, unsigned int height, bool hasMipmaps,
    bool createStorage)
    :
    TextureArray(numItems, format, 2, width, height, 1, hasMipmaps,
        createStorage)
{
    mType = GT_TEXTURE2_ARRAY;
}

unsigned int Texture2Array::GetWidth() const
{
    return TextureArray::GetDimension(0);
}

unsigned int Texture2Array::GetHeight() const
{
    return TextureArray::GetDimension(1);
}

