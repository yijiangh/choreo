// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTexture1Array.h>
using namespace gte;


Texture1Array::Texture1Array(unsigned int numItems, DFType format,
    unsigned int length, bool hasMipmaps, bool createStorage)
    :
    TextureArray(numItems, format, 1, length, 1, 1, hasMipmaps,
        createStorage)
{
    mType = GT_TEXTURE1_ARRAY;
}

unsigned int Texture1Array::GetLength() const
{
    return TextureArray::GetDimension(0);
}

