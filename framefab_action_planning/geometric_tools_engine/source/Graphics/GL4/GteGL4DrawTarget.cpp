// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4DrawTarget.h>
using namespace gte;

GL4DrawTarget::~GL4DrawTarget ()
{
    glDeleteFramebuffers(1, &mFrameBuffer);
}

GL4DrawTarget::GL4DrawTarget(DrawTarget const* target,
    std::vector<GL4TextureRT*>& rtTextures, GL4TextureDS* dsTexture)
    :
    GEDrawTarget(target),
    mRTTextures(rtTextures),
    mDSTexture(dsTexture),
    mFrameBuffer(0)
{
    LogAssert(target->GetNumTargets() <= rtTextures.size(),
        "DrawTargets has more targets than there are RT textures provided.");

    glGenFramebuffers(1, &mFrameBuffer);
}

std::shared_ptr<GEDrawTarget> GL4DrawTarget::Create(DrawTarget const* target,
    std::vector<GEObject*>& rtTextures, GEObject* dsTexture)
{
    std::vector<GL4TextureRT*> dxRTTextures(rtTextures.size());
    for (size_t i = 0; i < rtTextures.size(); ++i)
    {
        dxRTTextures[i] = static_cast<GL4TextureRT*>(rtTextures[i]);
    }
    GL4TextureDS* dxDSTexture = static_cast<GL4TextureDS*>(dsTexture);

    return std::make_shared<GL4DrawTarget>(target, dxRTTextures, dxDSTexture);
}

void GL4DrawTarget::Enable()
{
    // Save the current viewport settings so they can be restored when Disable is called.
    GLint intVals[4];
    GLdouble doubleVals[2];
    glGetIntegerv(GL_VIEWPORT, intVals);
    glGetDoublev(GL_DEPTH_RANGE, doubleVals);
    mSaveViewportX = intVals[0];
    mSaveViewportY = intVals[1];
    mSaveViewportWidth = intVals[2];
    mSaveViewportHeight = intVals[3];
    mSaveViewportNear = doubleVals[0];
    mSaveViewportFar = doubleVals[1];

    // Set viewport according to draw target;
    auto viewportWidth = static_cast<GLsizei>(mTarget->GetWidth());
    auto viewportHeight = static_cast<GLsizei>(mTarget->GetHeight());
    glViewport(0, 0, viewportWidth, viewportHeight);

    // Set depth range to full.
    glDepthRange(0.0, 1.0);

    // Bind the frame buffer.
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mFrameBuffer);

    // Attach depth buffer if there is one.
    if (mDSTexture)
    {
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, mDSTexture->GetGLHandle(), 0);
    }

    // Attach each render target.
    // Build list of attachments to use for drawing to.
    auto const numTargets = mTarget->GetNumTargets();
    std::vector<GLenum> useDrawBuffers(numTargets);
    for (unsigned i = 0; i < numTargets; ++i)
    {
        auto colorTarget = GL_COLOR_ATTACHMENT0 + i;

        useDrawBuffers[i] = colorTarget;

        auto textureRT = mRTTextures[i];
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, colorTarget, GL_TEXTURE_2D, textureRT->GetGLHandle(), 0);
    }

    // Set which draw buffers to use.
    glDrawBuffers(static_cast<GLsizei>(useDrawBuffers.size()), useDrawBuffers.data());
}

void GL4DrawTarget::Disable()
{
    // Restore to default frame buffer rendering.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Restore viewport.
    glViewport(mSaveViewportX, mSaveViewportY, mSaveViewportWidth, mSaveViewportHeight);
    glDepthRange(mSaveViewportNear, mSaveViewportFar);

    // When done, test each render target texture if it needs to have it's
    // mipmaps auto generated.
    auto const numTargets = mTarget->GetNumTargets();
    for (unsigned i = 0; i < numTargets; ++i)
    {
        auto textureRT = mRTTextures[i];
        if (textureRT->CanAutoGenerateMipmaps())
        {
            textureRT->GenerateMipmaps();
        }
    }
}
