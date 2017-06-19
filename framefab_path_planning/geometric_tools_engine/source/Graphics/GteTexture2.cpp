// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTexture2.h>
using namespace gte;


Texture2::Texture2(DFType format, unsigned int width, unsigned int height,
    bool hasMipmaps, bool createStorage)
    :
    TextureSingle(format, 2, width, height, 1, hasMipmaps, createStorage),
    mShared(false)
{
    mType = GT_TEXTURE2;
}

unsigned int Texture2::GetWidth() const
{
    return TextureSingle::GetDimension(0);
}

unsigned int Texture2::GetHeight() const
{
    return TextureSingle::GetDimension(1);
}

void Texture2::MakeShared()
{
    // Shared textures are required to be GPU writable.
    mUsage = SHADER_OUTPUT;
    mShared = true;
}

bool Texture2::IsShared() const
{
    return mShared;
}

