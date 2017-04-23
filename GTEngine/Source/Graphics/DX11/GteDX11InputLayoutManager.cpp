// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11InputLayoutManager.h>
using namespace gte;

DX11InputLayoutManager::~DX11InputLayoutManager()
{
    if (mMap.HasElements())
    {
        LogWarning("Input layout map is not empty on destruction.");
        UnbindAll();
    }
}

DX11InputLayoutManager::DX11InputLayoutManager()
{
}

DX11InputLayout* DX11InputLayoutManager::Bind(ID3D11Device* device,
    VertexBuffer const* vbuffer, Shader const* vshader)
{
    std::shared_ptr<DX11InputLayout> layout;
    if (vshader)
    {
        if (vbuffer)
        {
            if (!mMap.Get(std::make_pair(vbuffer, vshader), layout))
            {
                layout = std::make_shared<DX11InputLayout>(device, vbuffer, vshader);
                mMap.Insert(std::make_pair(vbuffer, vshader), layout);

#if defined(GTE_GRAPHICS_USE_NAMED_OBJECTS)
                std::string vbname = vbuffer->GetName();
                std::string vsname = vshader->GetName();
                if (vbname != "" || vsname != "")
                {
                    layout->SetName(vbname + " | " + vsname);
                }
#endif
            }
        }
        // else: A null vertex buffer is passed when an effect wants to
        // bypass the input assembler.
    }
    else
    {
        LogError("Vertex shader must be nonnull.");
    }
    return layout.get();
}

bool DX11InputLayoutManager::Unbind(VertexBuffer const* vbuffer)
{
    if (vbuffer)
    {
        std::vector<VBSPair> matches;
        mMap.GatherMatch(vbuffer, matches);
        for (auto match : matches)
        {
            std::shared_ptr<DX11InputLayout> layout;
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

bool DX11InputLayoutManager::Unbind(Shader const* vshader)
{
    if (vshader)
    {
        std::vector<VBSPair> matches;
        mMap.GatherMatch(vshader, matches);
        for (auto match : matches)
        {
            std::shared_ptr<DX11InputLayout> layout;
            mMap.Remove(match, layout);
        }
        return true;
    }
    else
    {
        LogError("Vertex shader must be nonnull.");
        return false;
    }
}

void DX11InputLayoutManager::UnbindAll()
{
    mMap.RemoveAll();
}

bool DX11InputLayoutManager::HasElements() const
{
    return mMap.HasElements();
}

DX11InputLayoutManager::LayoutMap::~LayoutMap()
{
}

DX11InputLayoutManager::LayoutMap::LayoutMap()
{
}

void DX11InputLayoutManager::LayoutMap::GatherMatch(
    VertexBuffer const* vbuffer, std::vector<VBSPair>& matches)
{
    mMutex.lock();
    {
        for (auto vbs : mMap)
        {
            if (vbuffer == vbs.first.first)
            {
                matches.push_back(vbs.first);
            }
        }
    }
    mMutex.unlock();
}

void DX11InputLayoutManager::LayoutMap::GatherMatch(Shader const* vshader,
    std::vector<VBSPair>& matches)
{
    mMutex.lock();
    {
        for (auto vbs : mMap)
        {
            if (vshader == vbs.first.second)
            {
                matches.push_back(vbs.first);
            }
        }
    }
    mMutex.unlock();
}
