// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTextureSingle.h>
using namespace gte;


TextureSingle::TextureSingle(DFType format, unsigned int numDimensions,
    unsigned int dim0, unsigned int dim1, unsigned int dim2, bool hasMipmaps,
    bool createStorage)
    :
    Texture(1, format, numDimensions, dim0, dim1, dim2, hasMipmaps,
        createStorage)
{
    mType = GT_TEXTURE_SINGLE;
}

