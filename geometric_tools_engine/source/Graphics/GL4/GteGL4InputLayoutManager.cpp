// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteGL4InputLayoutManager.h>
using namespace gte;

GL4InputLayoutManager::~GL4InputLayoutManager()
{
    if (mMap.HasElements())
    {
        LogWarning("Input layout map is not empty on destruction.");
        UnbindAll();
    }
}

GL4InputLayoutManager::GL4InputLayoutManager()
{
}

GL4InputLayout* GL4InputLayoutManager::Bind(GLuint programHandle,
    GLuint vbufferHandle, VertexBuffer const* vbuffer)
{
    std::shared_ptr<GL4InputLayout> layout;
    if (programHandle)
    {
        if (vbuffer)
        {
            if (!mMap.Get(std::make_pair(vbuffer, programHandle), layout))
            {
                layout = std::make_shared<GL4InputLayout>(programHandle, vbufferHandle, vbuffer);
                mMap.Insert(std::make_pair(vbuffer, programHandle), layout);
            }
        }
        // else: A null vertex buffer is passed when an effect wants to
        // bypass the input assembler.
    }
    else
    {
        LogError("Program must exist.");
    }
    return layout.get();
}

bool GL4InputLayoutManager::Unbind(VertexBuffer const* vbuffer)
{
    if (vbuffer)
    {
        std::vector<VBPPair> matches;
        mMap.GatherMatch(vbuffer, matches);
        for (auto match : matches)
        {
            std::shared_ptr<GL4InputLayout> layout;
            mMap.Remove(match, layout);
        }
        return true;
    }
    else
    {
        LogError("Vertex buffer must be nonnull.");
        return false;
    }
}

bool GL4InputLayoutManager::Unbind(Shader const*)
{
    return true;
}

void GL4InputLayoutManager::UnbindAll()
{
    mMap.RemoveAll();
}

bool GL4InputLayoutManager::HasElements() const
{
    return mMap.HasElements();
}

GL4InputLayoutManager::LayoutMap::~LayoutMap()
{
}

GL4InputLayoutManager::LayoutMap::LayoutMap()
{
}

void GL4InputLayoutManager::LayoutMap::GatherMatch(
    VertexBuffer const* vbuffer, std::vector<VBPPair>& matches)
{
    mMutex.lock();
    {
        for (auto vbp : mMap)
        {
            if (vbuffer == vbp.first.first)
            {
                matches.push_back(vbp.first);
            }
        }
    }
    mMutex.unlock();
}
