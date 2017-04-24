// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4TextureSingle.h>
using namespace gte;

GL4TextureSingle::~GL4TextureSingle()
{
    for (int level = 0; level < mNumLevels; ++level)
    {
        glDeleteBuffers(1, &mLevelPixelUnpackBuffer[level]);
        glDeleteBuffers(1, &mLevelPixelPackBuffer[level]);
    }
}

GL4TextureSingle::GL4TextureSingle(TextureSingle const* gtTexture, GLenum target, GLenum targetBinding)
    :
    GL4Texture(gtTexture, target, targetBinding)
{
    // Initially no staging buffers.
    std::fill(std::begin(mLevelPixelUnpackBuffer), std::end(mLevelPixelUnpackBuffer), 0);
    std::fill(std::begin(mLevelPixelPackBuffer), std::end(mLevelPixelPackBuffer), 0);
}

void GL4TextureSingle::Initialize()
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
    if (texture->GetData())
    {
        // Initialize with first mipmap level and then generate mipmaps.
        if (CanAutoGenerateMipmaps())
        {
            auto data = texture->GetDataFor(0);
            if (data)
            {
                LoadTextureLevel(0, data);
                GenerateMipmaps();
            }
        }
        // Initialize with each mipmap level.
        else
        {
            for (int level = 0; level < mNumLevels; ++level)
            {
                auto data = texture->GetDataFor(level);
                if (data)
                {
                    LoadTextureLevel(level, data);
                }
            }
        }
    }

    glBindTexture(mTarget, prevBinding);
}

bool GL4TextureSingle::Update()
{
    auto texture = GetTexture();

    // Only update the level 0 texture and then generate the mipmaps from there.
    if (CanAutoGenerateMipmaps())
    {
        if (!Update(0))
        {
            return false;
        }
        GenerateMipmaps();
    }

    // No auto gen mipmaps, so need to copy all of them to GPU.
    else
    {
        auto const numLevels = texture->GetNumLevels();
        for (unsigned int level = 0; level < numLevels; ++level)
        {
            if (!Update(level))
            {
                return false;
            }
        }
    }

    return true;
}

bool GL4TextureSingle::CopyCpuToGpu()
{
    auto texture = GetTexture();

    // Only update the level 0 texture and then generate the mipmaps from there.
    if (CanAutoGenerateMipmaps())
    {
        if (!CopyCpuToGpu(0))
        {
            return false;
        }
        GenerateMipmaps();
    }

    // No auto gen mipmaps, so need to copy all of them to GPU.
    else
    {
        auto const numLevels = texture->GetNumLevels();
        for (unsigned int level = 0; level < numLevels; ++level)
        {
            if (!CopyCpuToGpu(level))
            {
                return false;
            }
        }
    }

    return true;
}

bool GL4TextureSingle::CopyGpuToCpu()
{
    auto texture = GetTexture();
    auto const numLevels = texture->GetNumLevels();
    for (unsigned int level = 0; level < numLevels; ++level)
    {
        if (!CopyGpuToCpu(level))
        {
            return false;
        }
    }

    return true;
}

bool GL4TextureSingle::Update(unsigned int level)
{
    auto texture = GetTexture();
    if (texture->GetUsage() != Resource::DYNAMIC_UPDATE)
    {
        LogWarning("Texture usage is not DYNAMIC_UPDATE.");
        return false;
    }

    return DoCopyCpuToGpu(level);
}

bool GL4TextureSingle::CopyCpuToGpu(unsigned int level)
{
    if (!PreparedForCopy(GL_WRITE_ONLY))
    {
        return false;
    }

    return DoCopyCpuToGpu(level);
}

bool GL4TextureSingle::CopyGpuToCpu(unsigned int level)
{
    if (!PreparedForCopy(GL_READ_ONLY))
    {
        return false;
    }

    auto texture = GetTexture();

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

    auto data = texture->GetDataFor(level);
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

bool GL4TextureSingle::DoCopyCpuToGpu(unsigned int level)
{
    auto texture = GetTexture();

    // Cannot update automatically generated mipmaps in GPU.
    if (CanAutoGenerateMipmaps() && (level > 0))
    {
        LogWarning("Cannot update automatically generated mipmaps in GPU");
        return false;
    }

    // Make sure level is valid.
    auto const numLevels = texture->GetNumLevels();
    if (level >= numLevels)
    {
        LogWarning("Level for Texture is out of range");
        return false;
    }

    auto data = texture->GetDataFor(level);
    auto numBytes = texture->GetNumBytesFor(level);
    if ((nullptr == data) || (0 == numBytes))
    {
        LogWarning("No source data for Texture level=" + level);
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
        LoadTextureLevel(level, 0);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }
    else
    {
        LoadTextureLevel(level, data);
    }

    glBindTexture(target, 0);

    return true;
}

bool GL4TextureSingle::GenerateMipmaps()
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

void GL4TextureSingle::CreateStaging()
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
