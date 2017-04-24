// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4SamplerState.h>
using namespace gte;

GL4SamplerState::~GL4SamplerState()
{
    glDeleteSamplers(1, &mGLHandle);
}

GL4SamplerState::GL4SamplerState(SamplerState const* samplerState)
    :
    GL4DrawingState(samplerState)
{
    glGenSamplers(1, &mGLHandle);

    glSamplerParameteri(mGLHandle, GL_TEXTURE_WRAP_S, msMode[samplerState->mode[0]]);
    glSamplerParameteri(mGLHandle, GL_TEXTURE_WRAP_T, msMode[samplerState->mode[1]]);
    glSamplerParameteri(mGLHandle, GL_TEXTURE_WRAP_R, msMode[samplerState->mode[2]]);

    // TODO - GL_TEXTURE_MAX_ANISOTROPY_EXT is not defined?
    // glSamplerParameterf(samplerState, GL_TEXTURE_MAX_ANISOTROPY_EXT, samplerState->maxAnisotropy);

    glSamplerParameterf(mGLHandle, GL_TEXTURE_MIN_LOD, samplerState->minLOD);
    glSamplerParameterf(mGLHandle, GL_TEXTURE_MAX_LOD, samplerState->maxLOD);
    glSamplerParameterf(mGLHandle, GL_TEXTURE_LOD_BIAS, samplerState->mipLODBias);

    float borderColor[4];
    borderColor[0] = samplerState->borderColor[0];
    borderColor[1] = samplerState->borderColor[1];
    borderColor[2] = samplerState->borderColor[2];
    borderColor[3] = samplerState->borderColor[3];
    glSamplerParameterfv(mGLHandle, GL_TEXTURE_BORDER_COLOR, borderColor);

    switch(samplerState->filter)
    {
        case SamplerState::MIN_P_MAG_P_MIP_P:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case SamplerState::MIN_P_MAG_P_MIP_L:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case SamplerState::MIN_P_MAG_L_MIP_P:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        case SamplerState::MIN_P_MAG_L_MIP_L:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        case SamplerState::MIN_L_MAG_P_MIP_P:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case SamplerState::MIN_L_MAG_P_MIP_L:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case SamplerState::MIN_L_MAG_L_MIP_P:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        case SamplerState::MIN_L_MAG_L_MIP_L:
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        default:
            LogWarning("GL4 does not support samplerState filter = " + samplerState->filter);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MAG_FILTER, 0);
            glSamplerParameteri(mGLHandle, GL_TEXTURE_MIN_FILTER, 0);
            break;
    }
}

std::shared_ptr<GEObject> GL4SamplerState::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_SAMPLER_STATE)
    {
        return std::make_shared<GL4SamplerState>(
            static_cast<SamplerState const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}


GLint const GL4SamplerState::msMode[] =
{
    GL_REPEAT,          // WRAP
    GL_MIRRORED_REPEAT, // MIRROR
    GL_CLAMP_TO_EDGE,   // CLAMP
    GL_CLAMP_TO_BORDER, // BORDER
    GL_MIRRORED_REPEAT  // MIRROR_ONCE
};
