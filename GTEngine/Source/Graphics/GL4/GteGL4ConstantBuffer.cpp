// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4ConstantBuffer.h>
using namespace gte;

GL4ConstantBuffer::GL4ConstantBuffer(ConstantBuffer const* cbuffer)
    :
    GL4Buffer(cbuffer, GL_UNIFORM_BUFFER)
{
    Initialize();
}

std::shared_ptr<GEObject> GL4ConstantBuffer::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_CONSTANT_BUFFER)
    {
        return std::make_shared<GL4ConstantBuffer>(
            static_cast<ConstantBuffer const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void GL4ConstantBuffer::AttachToUnit(GLint uniformBufferUnit)
{
    glBindBufferBase(GL_UNIFORM_BUFFER, uniformBufferUnit, mGLHandle);
}
