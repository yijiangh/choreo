// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4TextureCubeArray.h>
using namespace gte;

GL4TextureCubeArray::~GL4TextureCubeArray()
{
    glDeleteTextures(1, &mGLHandle);
}

GL4TextureCubeArray::GL4TextureCubeArray(TextureCubeArray const* texture)
    :
    GL4TextureArray(texture, GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_BINDING_CUBE_MAP_ARRAY)
{
    // Create a texture structure.
    glGenTextures(1, &mGLHandle);
    glBindTexture(GL_TEXTURE_CUBE_MAP_ARRAY, mGLHandle);

    // Allocate (immutable) texture storage for all levels.
    auto const width = texture->GetDimension(0);
    auto const height = texture->GetDimension(1);
    auto const numItems = texture->GetNumItems();
    auto const numCubes = texture->GetNumCubes();
    glTexStorage3D(GL_TEXTURE_CUBE_MAP_ARRAY, mNumLevels, mInternalFormat, width, height, numItems);

    // The default is 4-byte alignment.  This allows byte alignment when data
    // from user buffers into textures and vice versa.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Set the range of levels.
    glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_MAX_LEVEL, mNumLevels-1);

    // Initialize with data?
    if (texture->GetData())
    {
        // Initialize with first mipmap level and then generate mipmaps.
        if (CanAutoGenerateMipmaps())
        {
            for (unsigned int cube = 0; cube < numCubes; ++cube)
            {
                for (unsigned int face = 0; face < texture->CubeFaceCount; ++face)
                {
                    auto data = texture->GetDataFor(cube, face, 0);
                    if (data)
                    {
                        auto item = texture->GetItemIndexFor(cube, face);
                        LoadTextureLevel(item, 0, data);
                    }
                }
            }
            GenerateMipmaps();
        }
        // Initialize with each mipmap level.
        else
        {
            for (unsigned int cube = 0; cube < numCubes; ++cube)
            {
                for (unsigned int face = 0; face < texture->CubeFaceCount; ++face)
                {
                    for (int level = 0; level < mNumLevels; ++level)
                    {
                        auto data = texture->GetDataFor(cube, face, level);
                        if (data)
                        {
                            auto item = texture->GetItemIndexFor(cube, face);
                            LoadTextureLevel(item, level, data);
                        }
                    }
                }
            }
        }
    }

    // Cannot leave this texture bound.
    glBindTexture(GL_TEXTURE_CUBE_MAP_ARRAY, 0);

    // Create a staging texture if requested.
    CreateStaging();
}

std::shared_ptr<GEObject> GL4TextureCubeArray::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE_CUBE_ARRAY)
    {
        return std::make_shared<GL4TextureCubeArray>(
            static_cast<TextureCubeArray const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

bool GL4TextureCubeArray::CanAutoGenerateMipmaps() const
{
    auto texture = GetTexture();

    return texture && texture->HasMipmaps() && texture->WantAutogenerateMipmaps();
}

void GL4TextureCubeArray::LoadTextureLevel(unsigned int item, unsigned int level, void const* data)
{
    auto texture = GetTexture();
    if (texture && level < texture->GetNumLevels())
    {
        auto const width = texture->GetDimensionFor(level, 0);
        auto const height = texture->GetDimensionFor(level, 1);

        // Determine cube and face indexes from the item index.
        auto const cube = texture->GetCubeIndexFor(item);
        auto const face = texture->GetFaceIndexFor(item);

        // Each face in the TextureCubeArray has a unique GL target.
        GLenum targetFace = msCubeFaceTarget[face];

        // For TextureCubeArray, use the 3D calls where the cube index is the third dimension.
        // Only updating one cube-face for the specified level.
        glTexSubImage3D(targetFace, level, 0, 0, cube, width, height, 1,
            mExternalFormat, mExternalType, data);
    }
}
