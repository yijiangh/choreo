// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteWrapper.h>
#include <Graphics/DX11/GteDX11Buffer.h>
using namespace gte;

DX11Buffer::DX11Buffer(Buffer const* buffer)
    :
    DX11Resource(buffer),
    mUpdateMapMode(D3D11_MAP_WRITE_DISCARD)
{
}

bool DX11Buffer::Update(ID3D11DeviceContext* context)
{
    Buffer* buffer = GetBuffer();
    if (buffer->GetUsage() != Resource::DYNAMIC_UPDATE)
    {
        LogWarning("Buffer usage is not DYNAMIC_UPDATE.");
        return false;
    }

    UINT numActiveBytes = buffer->GetNumActiveBytes();
    if (numActiveBytes > 0)
    {
        // Map the buffer.
        ID3D11Buffer* dxBuffer = GetDXBuffer();
        D3D11_MAPPED_SUBRESOURCE sub;
        HRESULT hr = context->Map(dxBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &sub);
        CHECK_HR_RETURN("Failed to map buffer", false);

        // Copy from CPU memory.
        if (mUpdateMapMode != D3D11_MAP_WRITE_DISCARD)
        {
            unsigned int offsetInBytes = buffer->GetOffset() * buffer->GetElementSize();
            char const* source = buffer->GetData() + offsetInBytes;
            char* target = (char*)sub.pData + offsetInBytes;
            Memcpy(target, source, numActiveBytes);
        }
        else
        {
            Memcpy(sub.pData, buffer->GetData(), buffer->GetNumBytes());
        }
        context->Unmap(dxBuffer, 0);
    }
    else
    {
        LogInformation("Buffer has zero active bytes.");
    }
    return true;
}

bool DX11Buffer::CopyCpuToGpu(ID3D11DeviceContext* context)
{
    if (!PreparedForCopy(D3D11_CPU_ACCESS_WRITE))
    {
        return false;
    }

    Buffer* buffer = GetBuffer();
    UINT numActiveBytes = buffer->GetNumActiveBytes();
    if (numActiveBytes > 0)
    {
        // Map the staging buffer.
        D3D11_MAPPED_SUBRESOURCE sub;
        HRESULT hr = context->Map(mStaging, 0, D3D11_MAP_WRITE, 0, &sub);
        CHECK_HR_RETURN("Failed to map staging buffer", false);

        // Copy from CPU memory to staging buffer.  For buffers, the
        // 'box' members are specified in number of bytes.  The inputs
        // 'DstX', 'DstY', and 'DstZ' are also specified in number of bytes.
        unsigned int offsetInBytes = buffer->GetOffset() * buffer->GetElementSize();
        char const* source = buffer->GetData() + offsetInBytes;
        char* target = (char*)sub.pData + offsetInBytes;
        Memcpy(target, source, numActiveBytes);
        context->Unmap(mStaging, 0);

        // Copy from staging buffer to GPU memory.
        //unsigned int offset = buffer->GetOffset();
        D3D11_BOX box = { offsetInBytes, 0, 0, offsetInBytes + numActiveBytes, 1, 1 };
        context->CopySubresourceRegion(
            GetDXBuffer(), 0, offsetInBytes, 0, 0, mStaging, 0, &box);
    }
    else
    {
        LogInformation("Buffer has zero active bytes.");
    }
    return true;
}

bool DX11Buffer::CopyGpuToCpu(ID3D11DeviceContext* context)
{
    if (!PreparedForCopy(D3D11_CPU_ACCESS_READ))
    {
        return false;
    }

    Buffer* buffer = GetBuffer();
    UINT numActiveBytes = buffer->GetNumActiveBytes();
    if (numActiveBytes > 0)
    {
        // Copy from GPU memory to staging buffer.
        unsigned int offsetInBytes = buffer->GetOffset() * buffer->GetElementSize();
        D3D11_BOX box = { offsetInBytes, 0, 0, offsetInBytes + numActiveBytes, 1, 1 };
        context->CopySubresourceRegion(
            mStaging, 0, offsetInBytes, 0, 0, GetDXBuffer(), 0, &box);

        // Map the staging buffer.
        D3D11_MAPPED_SUBRESOURCE sub;
        HRESULT hr = context->Map(mStaging, 0, D3D11_MAP_READ, 0, &sub);
        CHECK_HR_RETURN("Failed to map staging buffer", false);

        // Copy from staging buffer to CPU memory.
        char const* source = (char*)sub.pData + offsetInBytes;
        char* target = buffer->GetData() + offsetInBytes;
        Memcpy(target, source, numActiveBytes);
        context->Unmap(mStaging, 0);
    }
    else
    {
        LogInformation("Buffer has zero active bytes.");
    }
    return true;
}

void DX11Buffer::CopyGpuToGpu(ID3D11DeviceContext* context, ID3D11Resource* target)
{
    Buffer* buffer = GetBuffer();
    UINT numActiveBytes = buffer->GetNumActiveBytes();
    if (numActiveBytes > 0)
    {
        // Copy from GPU memory to staging buffer.
        unsigned int offsetInBytes = buffer->GetOffset() * buffer->GetElementSize();
        D3D11_BOX box = { offsetInBytes, 0, 0, offsetInBytes + numActiveBytes, 1, 1 };
        context->CopySubresourceRegion(
            target, 0, offsetInBytes, 0, 0, GetDXBuffer(), 0, &box);
    }
    else
    {
        LogInformation("Buffer has zero active bytes.");
    }
}

bool DX11Buffer::Update(ID3D11DeviceContext*, unsigned int)
{
    LogError("This was called polymorphically through DX11Resource.");
    return false;
}

bool DX11Buffer::CopyCpuToGpu(ID3D11DeviceContext*, unsigned int)
{
    LogError("This was called polymorphically through DX11Resource.");
    return false;
}

bool DX11Buffer::CopyGpuToCpu(ID3D11DeviceContext*, unsigned int)
{
    LogError("This was called polymorphically through DX11Resource.");
    return false;
}

void DX11Buffer::CopyGpuToGpu(ID3D11DeviceContext*, ID3D11Resource*, unsigned int)
{
    LogError("This was called polymorphically through DX11Resource.");
}

void DX11Buffer::CreateStaging(ID3D11Device* device,
    D3D11_BUFFER_DESC const& bf)
{
    D3D11_BUFFER_DESC desc;
    desc.ByteWidth = bf.ByteWidth;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = msStagingAccess[GetBuffer()->GetCopyType()];
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;
    desc.StructureByteStride = 0;

    HRESULT hr = device->CreateBuffer(&desc, nullptr,
        reinterpret_cast<ID3D11Buffer**>(&mStaging));
    CHECK_HR_RETURN_NONE("Failed to create staging buffer");
}

