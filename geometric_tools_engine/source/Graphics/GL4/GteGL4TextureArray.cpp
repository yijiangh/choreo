// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4TextureArray.h>
using namespace gte;

GL4TextureArray::~GL4TextureArray()
{
    for (int level = 0; level < mNumLevels; ++level)
    {
        glDeleteBuffers(1, &mLevelPixelUnpackBuffer[level]);
        glDeleteBuffers(1, &mLevelPixelPackBuffer[level]);
    }
}

GL4TextureArray::GL4TextureArray(TextureArray const* gtTexture, GLenum target, GLenum targetBinding)
    :
    GL4Texture(gtTexture, target, targetBinding)
{
    // Initially no staging buffers.
    std::fill(std::begin(mLevelPixelUnpackBuffer), std::end(mLevelPixelUnpackBuffer), 0);
    std::fill(std::begin(mLevelPixelPackBuffer), std::end(mLevelPixelPackBuffer), 0);
}

void GL4TextureArray::Initialize()
{
    // Save current binding for this texture target in order to restore it when done
    // since the gl texture object is needed to be bound to this texture target for
    // the operations to follow.
    GLint prevBinding;
    glGetIntegerv(mTargetBinding, &prevBinding);
    glBindTexture(mTarget, mGLHandle);

    // The default is 4-byte alignment.  This allows byte alignment when data
    // from user buffers into textures and vice versa.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Set the range of levels.
    glTexParameteri(mTarget, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(mTarget, GL_TEXTURE_MAX_LEVEL, mNumLevels-1);

    // Initialize with data?
    auto texture = GetTexture();
    auto const numItems = texture->GetNumItems();
    if (texture->GetData())
    {
        // Initialize with first mipmap level and then generate mipmaps.
        if (CanAutoGenerateMipmaps())
        {
            for (unsigned int item = 0; item < numItems; ++item)
            {
                auto data = texture->GetDataFor(item, 0);
                if (data)
                {
                    LoadTextureLevel(item, 0, data);
                }
            }
            GenerateMipmaps();
        }
        // Initialize with each mipmap level.
        else
        {
            for (unsigned int item = 0; item < numItems; ++item)
            {
                for (int level = 0; level < mNumLevels; ++level)
                {
                    auto data = texture->GetDataFor(item, level);
                    if (data)
                    {
                        LoadTextureLevel(item, level, data);
                    }
                }
            }
        }
    }

    glBindTexture(mTarget, prevBinding);
}

bool GL4TextureArray::Update()
{
    auto texture = GetTexture();
    auto const numItems = texture->GetNumItems();

    // Only update the level 0 texture and then generate the mipmaps from there.
    if (CanAutoGenerateMipmaps())
    {
        for (unsigned int item = 0; item < numItems; ++item)
        {
            if (!Update(item, 0))
            {
                return false;
            }
        }
        GenerateMipmaps();
    }

    // No auto gen mipmaps, so need to copy all of them to GPU.
    else
    {
        auto const numLevels = texture->GetNumLevels();
        for (unsigned int item = 0; item < numItems; ++item)
        {
            for (unsigned int level = 0; level < numLevels; ++level)
            {
                if (!Update(item, level))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool GL4TextureArray::CopyCpuToGpu()
{
    auto texture = GetTexture();
    auto const numItems = texture->GetNumItems();

    // Only update the level 0 texture and then generate the mipmaps from there.
    if (CanAutoGenerateMipmaps())
    {
        for (unsigned int item = 0; item < numItems; ++item)
        {
            if (!CopyCpuToGpu(item, 0))
            {
                return false;
            }
        }
        GenerateMipmaps();
    }

    // No auto gen mipmaps, so need to copy all of them to GPU.
    else
    {
        auto const numLevels = texture->GetNumLevels();
        for (unsigned int item = 0; item < numItems; ++item)
        {
            for (unsigned int level = 0; level < numLevels; ++level)
            {
                if (!CopyCpuToGpu(item, level))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool GL4TextureArray::CopyGpuToCpu()
{
    auto texture = GetTexture();
    auto const numItems = texture->GetNumItems();
    auto const numLevels = texture->GetNumLevels();
    for (unsigned int item = 0; item < numItems; ++item)
    {
        for (unsigned int level = 0; level < numLevels; ++level)
        {
            if (!CopyGpuToCpu(item, level))
            {
                return false;
            }
        }
    }

    return true;
}

bool GL4TextureArray::Update(unsigned int item, unsigned int level)
{
    auto texture = GetTexture();
    if (texture->GetUsage() != Resource::DYNAMIC_UPDATE)
    {
        LogWarning("Texture usage is not DYNAMIC_UPDATE.");
        return false;
    }

    return DoCopyCpuToGpu(item, level);
}

bool GL4TextureArray::CopyCpuToGpu(unsigned int item, unsigned int level)
{
    if (!PreparedForCopy(GL_WRITE_ONLY))
    {
        return false;
    }

    return DoCopyCpuToGpu(item, level);
}

bool GL4TextureArray::CopyGpuToCpu(unsigned int item, unsigned int level)
{
    if (!PreparedForCopy(GL_READ_ONLY))
    {
        return false;
    }

    auto texture = GetTexture();

    // Make sure item is valid.
    auto const numItems = texture->GetNumItems();
    if (item >= numItems)
    {
        LogWarning("Item for Texture is out of range");
        return false;
    }

    // Make sure level is valid.
    auto const numLevels = texture->GetNumLevels();
    if (level >= numLevels)
    {
        LogWarning("Level for Texture is out of range");
        return false;
    }

    auto pixBuffer = mLevelPixelPackBuffer[level];
    if (0 == pixBuffer)
    {
        LogWarning("Staging buffer not defined for level=" + level);
        return false;
    }

    auto data = texture->GetDataFor(item, level);
    auto numBytes = texture->GetNumBytesFor(level);
    if ((nullptr == data) || (0 == numBytes))
    {
        LogWarning("No target data for Texture level=" + level);
        return false;
    }

    auto const target = GetTarget();
    glBindTexture(target, mGLHandle);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixBuffer);
    glGetTexImage(target, level, mExternalFormat, mExternalType, 0);
    glGetBufferSubData(GL_PIXEL_PACK_BUFFER, 0, numBytes, data);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    glBindTexture(target, 0);

    return true;
}

bool GL4TextureArray::DoCopyCpuToGpu(unsigned int item, unsigned int level)
{
    auto texture = GetTexture();

    // Cannot update automatically generated mipmaps in GPU.
    if (CanAutoGenerateMipmaps() && (level > 0))
    {
        LogWarning("Cannot update automatically generated mipmaps in GPU");
        return false;
    }

    // Make sure item is valid.
    auto const numItems = texture->GetNumItems();
    if (item >= numItems)
    {
        LogWarning("Item for TextureArray is out of range");
        return false;
    }

    // Make sure level is valid.
    auto const numLevels = texture->GetNumLevels();
    if (level >= numLevels)
    {
        LogWarning("Level for TextureArray is out of range");
        return false;
    }

    auto data = texture->GetDataFor(item, level);
    auto numBytes = texture->GetNumBytesFor(level);
    if ((nullptr == data) || (0 == numBytes))
    {
        LogWarning("No source data for TextureArray level=" + level);
        return false;
    }

    auto const target = GetTarget();
    glBindTexture(target, mGLHandle);

    // Use staging buffer if present.
    auto pixBuffer = mLevelPixelUnpackBuffer[level];
    if (0 != pixBuffer)
    {
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pixBuffer);
        glBufferSubData(GL_PIXEL_UNPACK_BUFFER, 0, numBytes, data);
        LoadTextureLevel(item, level, 0);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }
    else
    {
        LoadTextureLevel(item, level, data);
    }

    glBindTexture(target, 0);

    return true;
}

void GL4TextureArray::CreateStaging()
{
    auto texture = GetTexture();
    auto const copyType = texture->GetCopyType();

    auto const createPixelUnpackBuffers = 
        (copyType == Resource::COPY_CPU_TO_STAGING) ||
        (copyType == Resource::COPY_BIDIRECTIONAL);

    auto const createPixelPackBuffers =
        (copyType == Resource::COPY_STAGING_TO_CPU) ||
        (copyType == Resource::COPY_BIDIRECTIONAL);

    // TODO
    // Determine frequency and nature of usage for this staging
    // buffer when created by calling glBufferData.
    GLenum usage = GL_DYNAMIC_DRAW;

    if (createPixelUnpackBuffers)
    {
        for (int level = 0; level < mNumLevels; ++level)
        {
            auto& pixBuffer = mLevelPixelUnpackBuffer[level];
            if (0 == pixBuffer)
            {
                auto numBytes = texture->GetNumBytesFor(level);

                // Create pixel buffer for staging.
                glGenBuffers(1, &pixBuffer);
                glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pixBuffer);
                glBufferData(GL_PIXEL_UNPACK_BUFFER, numBytes, 0, usage);
                glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
            }
        }
    }

    if (createPixelPackBuffers)
    {
        for (int level = 0; level < mNumLevels; ++level)
        {
            auto& pixBuffer = mLevelPixelPackBuffer[level];
            if (0 == pixBuffer)
            {
                auto numBytes = texture->GetNumBytesFor(level);

                // Create pixel buffer for staging.
                glGenBuffers(1, &pixBuffer);
                glBindBuffer(GL_PIXEL_PACK_BUFFER, pixBuffer);
                glBufferData(GL_PIXEL_PACK_BUFFER, numBytes, 0, usage);
                glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            }
        }
    }
}

bool GL4TextureArray::GenerateMipmaps()
{
    if (CanAutoGenerateMipmaps())
    {
        // Save current binding for this texture target in order to restore it when done
        // since the gl texture object is needed to be bound to this texture target for
        // the operations to follow.
        GLint prevBinding;
        glGetIntegerv(mTargetBinding, &prevBinding);
        glBindTexture(mTarget, mGLHandle);

        // Generate the mipmaps.
        // All of this binding save and restore is not necessary in OpenGL 4.5
        // where glGenerateTextureMipamap(mGLHandle) can simply be used.
        glGenerateMipmap(mTarget);

        glBindTexture(mTarget, prevBinding);

        return true;
    }
    return false;
}


GLenum const GL4TextureArray::msCubeFaceTarget[6] =
{
    GL_TEXTURE_CUBE_MAP_POSITIVE_X, // CubeFacePositiveX
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X, // CubeFaceNegativeX
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y, // CubeFacePositiveY
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, // CubeFaceNegativeY
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z, // CubeFacePositiveZ
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z  // CubeFaceNegativeZ
};
