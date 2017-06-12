// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4Texture2.h>
using namespace gte;

GL4Texture2::~GL4Texture2()
{
    glDeleteTextures(1, &mGLHandle);
}

GL4Texture2::GL4Texture2(Texture2 const* texture)
    :
    GL4TextureSingle(texture, GL_TEXTURE_2D, GL_TEXTURE_BINDING_2D)
{
    // Create a texture structure.
    glGenTextures(1, &mGLHandle);
    glBindTexture(GL_TEXTURE_2D, mGLHandle);

    // Allocate (immutable) texture storage for all levels.
    auto const width = texture->GetDimension(0);
    auto const height = texture->GetDimension(1);
    glTexStorage2D(GL_TEXTURE_2D, mNumLevels, mInternalFormat, width, height);

    Initialize();

    // Cannot leave this texture bound.
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create a staging texture if requested.
    CreateStaging();
}

std::shared_ptr<GEObject> GL4Texture2::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE2)
    {
        return std::make_shared<GL4Texture2>(
            static_cast<Texture2 const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

bool GL4Texture2::CanAutoGenerateMipmaps() const
{
    auto texture = GetTexture();

    return texture &&
        texture->HasMipmaps() &&
        texture->WantAutogenerateMipmaps() &&
        !texture->IsShared();
}

void GL4Texture2::LoadTextureLevel(unsigned int level, void const* data)
{
    auto texture = GetTexture();
    if (texture && level < texture->GetNumLevels())
    {
        auto width = texture->GetDimension(0);
        auto height = texture->GetDimension(1);

        glTexSubImage2D(GL_TEXTURE_2D, level, 0, 0, width, height,
            mExternalFormat, mExternalType, data);
    }
}
