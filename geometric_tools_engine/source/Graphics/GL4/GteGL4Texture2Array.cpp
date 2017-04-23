// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4Texture2Array.h>
using namespace gte;

GL4Texture2Array::~GL4Texture2Array()
{
    glDeleteTextures(1, &mGLHandle);
}

GL4Texture2Array::GL4Texture2Array(Texture2Array const* texture)
    :
    GL4TextureArray(texture, GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BINDING_2D_ARRAY)
{
    // Create a texture structure.
    glGenTextures(1, &mGLHandle);
    glBindTexture(GL_TEXTURE_2D_ARRAY, mGLHandle);

    // Allocate (immutable) texture storage for all levels.
    auto const width = texture->GetDimension(0);
    auto const height = texture->GetDimension(1);
    auto const numItems = texture->GetNumItems();
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, mNumLevels, mInternalFormat, width, height, numItems);

    Initialize();

    // Cannot leave this texture bound.
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

    // Create a staging texture if requested.
    CreateStaging();
}

std::shared_ptr<GEObject> GL4Texture2Array::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE2_ARRAY)
    {
        return std::make_shared<GL4Texture2Array>(
            static_cast<Texture2Array const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

bool GL4Texture2Array::CanAutoGenerateMipmaps() const
{
    auto texture = GetTexture();

    return texture && texture->HasMipmaps() && texture->WantAutogenerateMipmaps();
}

void GL4Texture2Array::LoadTextureLevel(unsigned int item, unsigned int level, void const* data)
{
    auto texture = GetTexture();
    if (texture && level < texture->GetNumLevels())
    {
        auto const width = texture->GetDimensionFor(level, 0);
        auto const height = texture->GetDimensionFor(level, 1);

        // For Texture2Array, use the 3D calls where the slice (or item) is the third dimension.
        // Only updating one slice for the specified level.
        glTexSubImage3D(GL_TEXTURE_2D_ARRAY, level, 0, 0, item, width, height, 1,
            mExternalFormat, mExternalType, data);
    }
}
