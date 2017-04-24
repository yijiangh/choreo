// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4StructuredBuffer.h>
#include <algorithm>
using namespace gte;

GL4StructuredBuffer::GL4StructuredBuffer(StructuredBuffer const* cbuffer)
    :
    GL4Buffer(cbuffer, GL_SHADER_STORAGE_BUFFER),
    mCounterOffset(0)
{
    Initialize();
}

std::shared_ptr<GEObject> GL4StructuredBuffer::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_STRUCTURED_BUFFER)
    {
        return std::make_shared<GL4StructuredBuffer>(
            static_cast<StructuredBuffer const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void GL4StructuredBuffer::AttachToUnit(GLint shaderStorageBufferUnit)
{
    auto buffer = GetStructuredBuffer();

    // Cannot use glBindBufferBase because if structured buffer has a counter
    // associated with it, then there are extra bytes allocated in the buffer
    // to store the counter value.  The structured buffer data does start
    // at offset=0, so all that is needed is the actual number of data bytes
    // in the StructuredBuffer object.
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, shaderStorageBufferUnit, mGLHandle, 0, buffer->GetNumBytes());
}

bool GL4StructuredBuffer::CopyCounterValueToBuffer(GL4Buffer* targetBuffer, GLint offset)
{
    if (!targetBuffer)
    {
        return false;
    }

    auto buffer = GetStructuredBuffer();
    if (StructuredBuffer::CT_NONE == buffer->GetCounterType())
    {
        return false;
    }

    glBindBuffer(GL_COPY_READ_BUFFER, mGLHandle);
    glBindBuffer(GL_COPY_WRITE_BUFFER, targetBuffer->GetGLHandle());
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, mCounterOffset, offset, 4);
    return true;
}

bool GL4StructuredBuffer::CopyCounterValueFromBuffer(GL4Buffer* sourceBuffer, GLint offset)
{
    if (!sourceBuffer)
    {
        return false;
    }

    auto buffer = GetStructuredBuffer();
    if (StructuredBuffer::CT_NONE == buffer->GetCounterType())
    {
        return false;
    }

    glBindBuffer(GL_COPY_READ_BUFFER, sourceBuffer->GetGLHandle());
    glBindBuffer(GL_COPY_WRITE_BUFFER, mGLHandle);
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, offset, mCounterOffset, 4);
    return true;
}

bool GL4StructuredBuffer::GetNumActiveElements()
{
    auto buffer = GetStructuredBuffer();
    if (StructuredBuffer::CT_NONE == buffer->GetCounterType())
    {
        return false;
    }

    // Read the count from the location in the buffer past the
    // structured buffer data.
    GLint count;
    glBindBuffer(mType, mGLHandle);
    glGetBufferSubData(mType, mCounterOffset, 4, &count);
    glBindBuffer(mType, 0);

    count = (std::max)(0, count);
    buffer->SetNumActiveElements(count);

    return true;
}

bool GL4StructuredBuffer::SetNumActiveElements()
{
    auto buffer = GetStructuredBuffer();
    if (StructuredBuffer::CT_NONE == buffer->GetCounterType())
    {
        return false;
    }

    // Get count from front end structured buffer object.
    if (!buffer->GetKeepInternalCount())
    {
        GLint count = buffer->GetNumActiveElements();

        glBindBuffer(mType, mGLHandle);
        glBufferSubData(mType, mCounterOffset, 4, &count);
        glBindBuffer(mType, 0);
    }

    return true;
}

bool GL4StructuredBuffer::CopyGpuToCpu()
{
    auto buffer = GetStructuredBuffer();

    // Need to read number of active elements first if there is
    // a counter attached to this structured buffer.
    if (StructuredBuffer::CT_NONE != buffer->GetCounterType())
    {
        if (!GetNumActiveElements())
        {
            return false;
        }
    }

    return GL4Buffer::CopyGpuToCpu();
}

void GL4StructuredBuffer::Initialize()
{
    auto buffer = GetStructuredBuffer();

    // Regular structured buffer (no counter)?
    if (StructuredBuffer::CT_NONE == buffer->GetCounterType())
    {
        GL4Buffer::Initialize();
    }

    // Structured buffer has a counter (any type)?
    // Allocate extra bytes to store the counter value.
    else
    {
        glBindBuffer(mType, mGLHandle);

        // How many bytes are needed for the structured buffer data.
        auto numBytes = buffer->GetNumBytes();

        // Allocate extra bytes to align to 4 byte boundary for the offset.
        // According to glBufferSubData:
        // "Clients must align data elements consistent with the requirements
        // of the client platform, with an additional base-level requirement
        // that an offset within a buffer to a datum comprising N bytes be a
        // multiple of N."
        numBytes = ((numBytes + 3) / 4) * 4;
        mCounterOffset = numBytes;

        // Allocate 4 extra bytes for the counter itself.
        numBytes += 4;

        // Create a dynamic buffer that allows calls to glBufferSubData to
        // update the buffer and the associated counter separately but within
        // the same buffer contents.
        glBufferData(mType, numBytes, nullptr, GL_DYNAMIC_DRAW);

        // Initialize the GPU memory from the buffer.
        auto data = buffer->GetData();
        if (data)
        {
            glBufferSubData(mType, 0, buffer->GetNumBytes(), buffer->GetData());
        }

        // Initialize the count value.
        GLint count = buffer->GetNumElements();
        glBufferSubData(mType, mCounterOffset, 4, &count);

        glBindBuffer(mType, 0);
    }
}
