// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteBaseEngine.h>
using namespace gte;

BaseEngine::~BaseEngine()
{
}

BaseEngine::BaseEngine()
    :
    mXSize(0),
    mYSize(0),
    mClearDepth(1.0f),
    mClearStencil(0)
{
    mClearColor.fill(1.0f);
}

void BaseEngine::SetFont(std::shared_ptr<Font> const& font)
{
    mActiveFont = font;
}

void BaseEngine::CreateDefaultGlobalState()
{
    mDefaultBlendState = std::make_shared<BlendState>();
    mDefaultDepthStencilState = std::make_shared<DepthStencilState>();
    mDefaultRasterizerState = std::make_shared<RasterizerState>();

#if defined(GTE_GRAPHICS_USE_NAMED_OBJECTS)
    mDefaultBlendState->SetName("BaseEngine::mDefaultBlendState");
    mDefaultDepthStencilState->SetName("BaseEngine::mDefaultDepthStencilState");
    mDefaultRasterizerState->SetName("BaseEngine::mDefaultRasterizerState");
#endif

    SetDefaultBlendState();
    SetDefaultDepthStencilState();
    SetDefaultRasterizerState();
}

void BaseEngine::DestroyDefaultGlobalState()
{
    mDefaultBlendState = nullptr;
    mActiveBlendState = nullptr;
    mDefaultDepthStencilState = nullptr;
    mActiveDepthStencilState = nullptr;
    mDefaultRasterizerState = nullptr;
    mActiveRasterizerState = nullptr;
}
