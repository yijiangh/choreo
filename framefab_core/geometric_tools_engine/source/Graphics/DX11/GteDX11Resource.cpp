// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11Resource.h>
using namespace gte;

DX11Resource::~DX11Resource()
{
    FinalRelease(mStaging);
}

DX11Resource::DX11Resource(Resource const* gtResource)
    :
    DX11GraphicsObject(gtResource),
    mStaging(nullptr)
{
    // Derived classes must create the staging resource, because DX11 does
    // not have a generic description structure that could be used here
    // otherwise.
}

D3D11_MAPPED_SUBRESOURCE DX11Resource::MapForWrite(ID3D11DeviceContext* context, unsigned int sri)
{
    D3D11_MAPPED_SUBRESOURCE mapped;
    HRESULT hr = context->Map(GetDXResource(), sri, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
    if (FAILED(hr))
    {
        mapped.pData = nullptr;
        mapped.RowPitch = 0;
        mapped.DepthPitch = 0;
    }
    return mapped;
}

void DX11Resource::Unmap(ID3D11DeviceContext* context, unsigned int sri)
{
    context->Unmap(GetDXResource(), sri);
}

void DX11Resource::SetName(std::string const& name)
{
    DX11GraphicsObject::SetName(name);
    HRESULT hr = SetPrivateName(mStaging, name);
    CHECK_HR_RETURN_NONE("Failed to set private name");
}

bool DX11Resource::PreparedForCopy(D3D11_CPU_ACCESS_FLAG access) const
{
    // Verify existence of objects.
    if (!mDXObject)
    {
        LogError("Resource does not exist.");
        return false;
    }
    if (!mStaging)
    {
        LogError("Staging resource does not exist.");
        return false;
    }

    // Verify the copy type.
    if ((msStagingAccess[GetResource()->GetCopyType()] & access) == 0)
    {
        LogError("Staging resource has incorrect CPU access.");
        return false;
    }

    return true;
}


UINT const DX11Resource::msStagingAccess[] =
{
    D3D11_CPU_ACCESS_NONE,          // COPY_NONE
    D3D11_CPU_ACCESS_WRITE,         // COPY_CPU_TO_STAGING
    D3D11_CPU_ACCESS_READ,          // COPY_STAGING_TO_CPU
    D3D11_CPU_ACCESS_READ_WRITE     // COPY_BIDIRECTIONAL
};
