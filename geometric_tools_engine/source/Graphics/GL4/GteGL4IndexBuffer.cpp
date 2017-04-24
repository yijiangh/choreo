// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4IndexBuffer.h>
using namespace gte;

GL4IndexBuffer::GL4IndexBuffer(IndexBuffer const* ibuffer)
    :
    GL4Buffer(ibuffer, GL_ELEMENT_ARRAY_BUFFER)
{
    Initialize();
}

std::shared_ptr<GEObject> GL4IndexBuffer::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_INDEX_BUFFER)
    {
        return std::make_shared<GL4IndexBuffer>(
            static_cast<IndexBuffer const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void GL4IndexBuffer::Enable()
{
    glBindBuffer(mType, mGLHandle);
}

void GL4IndexBuffer::Disable()
{
    glBindBuffer(mType, 0);
}
