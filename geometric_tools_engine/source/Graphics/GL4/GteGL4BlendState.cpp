// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4BlendState.h>
using namespace gte;

GL4BlendState::GL4BlendState(BlendState const* blendState)
    :
    GL4DrawingState(blendState)
{
    mEnableAlphaToCoverage = blendState->enableAlphaToCoverage;
    mEnableIndependentBlend = blendState->enableIndependentBlend;
    for (unsigned int i = 0; i < BlendState::NUM_TARGETS; ++i)
    {
        BlendState::Target const& in = blendState->target[i];
        Target& out = mTarget[i];
        out.enable = (in.enable ? GL_TRUE : GL_FALSE);
        out.srcColor = msMode[in.srcColor];
        out.dstColor = msMode[in.dstColor];
        out.opColor = msOperation[in.opColor];
        out.srcAlpha = msMode[in.srcAlpha];
        out.dstAlpha = msMode[in.dstAlpha];
        out.opAlpha = msOperation[in.opAlpha];
        out.rMask = (in.mask & 1 ? GL_TRUE : GL_FALSE);
        out.gMask = (in.mask & 2 ? GL_TRUE : GL_FALSE);
        out.bMask = (in.mask & 4 ? GL_TRUE : GL_FALSE);
        out.aMask = (in.mask & 8 ? GL_TRUE : GL_FALSE);
    }
    mBlendColor = blendState->blendColor;
    mSampleMask = blendState->sampleMask;
}

std::shared_ptr<GEObject> GL4BlendState::Create(void*, GraphicsObject const* object)
{
    if (object->GetType() == GT_BLEND_STATE)
    {
        return std::make_shared<GL4BlendState>(
            static_cast<BlendState const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void GL4BlendState::Enable()
{
    if (mEnableAlphaToCoverage)
    {
        glEnable(GL_SAMPLE_COVERAGE);
    }
    else
    {
        glDisable(GL_SAMPLE_COVERAGE);
    }

    if (mEnableIndependentBlend)
    {
        for (unsigned int i = 0; i < BlendState::NUM_TARGETS; ++i)
        {
            Target const& target = mTarget[i];
            if (target.enable)
            {
                glEnable(GL_BLEND);
                glBlendFuncSeparatei(i, target.srcColor, target.dstColor,
                    target.srcAlpha, target.dstAlpha);
                glBlendEquationSeparatei(i, target.opColor, target.opAlpha);
            }
            else
            {
                glDisable(GL_BLEND);
            }
            glColorMaski(i, target.rMask, target.gMask, target.bMask, target.aMask);
            glSampleMaski(i, mSampleMask);
        }
    }
    else
    {
        Target const& target = mTarget[0];
        if (target.enable)
        {
            glEnable(GL_BLEND);
            glBlendFuncSeparate(target.srcColor, target.dstColor,
                target.srcAlpha, target.dstAlpha);
            glBlendEquationSeparate(target.opColor, target.opAlpha);
        }
        else
        {
            glDisable(GL_BLEND);
        }
        glColorMask(target.rMask, target.gMask, target.bMask, target.aMask);
        glSampleMaski(0, mSampleMask);
    }

    glBlendColor(mBlendColor[0], mBlendColor[1], mBlendColor[2], mBlendColor[3]);
}


GLenum const GL4BlendState::msMode[] =
{
    GL_ZERO,
    GL_ONE,
    GL_SRC_COLOR,
    GL_ONE_MINUS_SRC_COLOR,
    GL_SRC_ALPHA,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_DST_ALPHA,
    GL_ONE_MINUS_DST_ALPHA,
    GL_DST_COLOR,
    GL_ONE_MINUS_DST_COLOR,
    GL_SRC_ALPHA_SATURATE,
    GL_CONSTANT_COLOR,
    GL_ONE_MINUS_CONSTANT_COLOR,
    GL_SRC1_COLOR,
    GL_ONE_MINUS_SRC1_COLOR,
    GL_SRC1_ALPHA,
    GL_ONE_MINUS_SRC1_ALPHA
};

GLenum const GL4BlendState::msOperation[] =
{
    GL_FUNC_ADD,
    GL_FUNC_SUBTRACT,
    GL_FUNC_REVERSE_SUBTRACT,
    GL_MIN,
    GL_MAX
};
