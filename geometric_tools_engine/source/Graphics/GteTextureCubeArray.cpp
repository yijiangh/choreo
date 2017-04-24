// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTextureCubeArray.h>
using namespace gte;


TextureCubeArray::TextureCubeArray(unsigned int numCubes, DFType format,
    unsigned int length, bool hasMipmaps, bool createStorage)
    :
    TextureArray(CubeFaceCount * numCubes, format, 2, length, length, 1, hasMipmaps,
        createStorage),
    mNumCubes(numCubes)
{
    mType = GT_TEXTURE_CUBE_ARRAY;
}

unsigned int TextureCubeArray::GetNumCubes() const
{
    return mNumCubes;
}

unsigned int TextureCubeArray::GetLength() const
{
    return TextureArray::GetDimension(0);
}

