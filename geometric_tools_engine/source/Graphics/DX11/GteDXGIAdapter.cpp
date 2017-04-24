// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDXGIAdapter.h>
using namespace gte;


DXGIAdapter::~DXGIAdapter()
{
    if (mAdapter)
    {
        mAdapter->Release();
    }
}

DXGIAdapter::DXGIAdapter(DXGIAdapter const& object)
    :
    mAdapter(nullptr)
{
    *this = object;
}

DXGIAdapter::DXGIAdapter(IDXGIAdapter1* adapter)
    :
    mAdapter(adapter)
{
    ZeroMemory(&mDescription, sizeof(DXGI_ADAPTER_DESC1));
    if (mAdapter)
    {
        HRESULT hr = mAdapter->GetDesc1(&mDescription);
        CHECK_HR_RETURN_NONE("Unexpected error");

        for (UINT i = 0; /**/; ++i)
        {
            IDXGIOutput* output = nullptr;
            hr = mAdapter->EnumOutputs(i, &output);
            if (hr != DXGI_ERROR_NOT_FOUND)
            {
                mOutputs.push_back(DXGIOutput(output));
            }
            else
            {
                break;
            }
        }
    }
}

DXGIAdapter& DXGIAdapter::operator=(DXGIAdapter const& object)
{
    if (object.mAdapter)
    {
        object.mAdapter->AddRef();
    }

    SafeRelease(mAdapter);

    mAdapter = object.mAdapter;
    mDescription = object.mDescription;
    mOutputs = object.mOutputs;
    return *this;
}

IDXGIAdapter1* DXGIAdapter::GetAdapter() const
{
    return mAdapter;
}

DXGI_ADAPTER_DESC1 const& DXGIAdapter::GetDescription() const
{
    return mDescription;
}

std::vector<DXGIOutput> const& DXGIAdapter::GetOutputs() const
{
    return mOutputs;
}

void DXGIAdapter::Enumerate(std::vector<DXGIAdapter>& adapters)
{
    adapters.clear();

    IDXGIFactory1* factory = nullptr;
    HRESULT hr = CreateDXGIFactory1(__uuidof(IDXGIFactory1),
        (void**)&factory);
    CHECK_HR_RETURN_NONE("Unexpected error");

    if (factory)
    {
        for (UINT i = 0; /**/; ++i)
        {
            IDXGIAdapter1* adapter = nullptr;
            hr = factory->EnumAdapters1(i, &adapter);
            if (hr != DXGI_ERROR_NOT_FOUND)
            {
                adapters.push_back(DXGIAdapter(adapter));
            }
            else
            {
                break;
            }
        }

        SafeRelease(factory);
    }
}

