// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11StructuredBuffer.h>
using namespace gte;

DX11StructuredBuffer::~DX11StructuredBuffer()
{
    FinalRelease(mSRView);
    FinalRelease(mUAView);
    FinalRelease(mCounterStaging);
}

DX11StructuredBuffer::DX11StructuredBuffer(ID3D11Device* device,
    StructuredBuffer const* sbuffer)
    :
    DX11Buffer(sbuffer),
    mSRView(nullptr),
    mUAView(nullptr),
    mCounterStaging(nullptr)
{
    // Specify the buffer description.
    D3D11_BUFFER_DESC desc;
    desc.ByteWidth = sbuffer->GetNumBytes();
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = sbuffer->GetElementSize();
    Resource::Usage usage = sbuffer->GetUsage();
    if (usage == Resource::IMMUTABLE)
    {
        desc.Usage = D3D11_USAGE_IMMUTABLE;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
    }
    else if (usage == Resource::DYNAMIC_UPDATE)
    {
        desc.Usage = D3D11_USAGE_DYNAMIC;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    }
    else  // usage == Resource::SHADER_OUTPUT
    {
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
    }

    // Create the buffer.
    ID3D11Buffer* buffer = nullptr;
    HRESULT hr;
    if (sbuffer->GetData())
    {
        D3D11_SUBRESOURCE_DATA data;
        data.pSysMem = sbuffer->GetData();
        data.SysMemPitch = 0;
        data.SysMemSlicePitch = 0;
        hr = device->CreateBuffer(&desc, &data, &buffer);
    }
    else
    {
        hr = device->CreateBuffer(&desc, nullptr, &buffer);
    }
    CHECK_HR_RETURN_NONE("Failed to create structured buffer");
    mDXObject = buffer;

    // Create views of the buffer.
    CreateSRView(device);
    if (usage == Resource::SHADER_OUTPUT)
    {
        CreateUAView(device);
    }

    // Create a staging buffer if requested.
    if (sbuffer->GetCopyType() != Resource::COPY_NONE)
    {
        CreateStaging(device, desc);
    }

    // Create a staging buffer for the internal counter.
    if (sbuffer->GetCounterType() != StructuredBuffer::CT_NONE)
    {
        CreateCounterStaging(device);
    }
}

std::shared_ptr<GEObject> DX11StructuredBuffer::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_STRUCTURED_BUFFER)
    {
        return std::make_shared<DX11StructuredBuffer>(
            reinterpret_cast<ID3D11Device*>(device),
            static_cast<StructuredBuffer const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

bool DX11StructuredBuffer::CopyGpuToCpu(ID3D11DeviceContext* context)
{
    if (mCounterStaging)
    {
        if (!GetNumActiveElements(context))
        {
            return false;
        }
    }
    return DX11Buffer::CopyGpuToCpu(context);
}

bool DX11StructuredBuffer::GetNumActiveElements(
    ID3D11DeviceContext* context)
{
    // Copy the number of active elements from GPU to staging buffer.
    context->CopyStructureCount(mCounterStaging, 0, mUAView);

    // Map the staging buffer.
    D3D11_MAPPED_SUBRESOURCE sub;
    HRESULT hr = context->Map(mCounterStaging, 0, D3D11_MAP_READ, 0, &sub);
    CHECK_HR_RETURN("Failed to map counter staging buffer", false);

    // Get the number of active elements in the buffer.  The internal counter
    // appears to increment even when the buffer is full, so it needs to be
    // clamped to the maximum value.  The clamping occurs in the call to
    // SetNumActiveElements().
    unsigned int numActive = *static_cast<unsigned int*>(sub.pData);
    context->Unmap(mCounterStaging, 0);

    // Copy the number to the CPU.
    GetStructuredBuffer()->SetNumActiveElements(numActive);
    return true;
}

void DX11StructuredBuffer::SetName(std::string const& name)
{
    DX11Buffer::SetName(name);
    HRESULT hr = SetPrivateName(mSRView, name);
    CHECK_HR_RETURN_NONE("Failed to set private name");
    hr = SetPrivateName(mUAView, name);
    CHECK_HR_RETURN_NONE("Failed to set private name");
    if (mCounterStaging)
    {
        hr = SetPrivateName(mCounterStaging, name);
        CHECK_HR_RETURN_NONE("Failed to set private name");
    }
}

void DX11StructuredBuffer::CreateSRView(ID3D11Device* device)
{
    ID3D11Buffer* buffer = GetDXBuffer();
    StructuredBuffer* sbuffer = GetStructuredBuffer();

    D3D11_SHADER_RESOURCE_VIEW_DESC desc;
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = sbuffer->GetOffset();
    desc.Buffer.NumElements = sbuffer->GetNumElements();

    HRESULT hr = device->CreateShaderResourceView(buffer, &desc, &mSRView);
    CHECK_HR_RETURN_NONE("Failed to create shader resource view");
}

void DX11StructuredBuffer::CreateUAView(ID3D11Device* device)
{
    ID3D11Buffer* buffer = GetDXBuffer();
    StructuredBuffer* sbuffer = GetStructuredBuffer();

    D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = sbuffer->GetOffset();
    desc.Buffer.NumElements = sbuffer->GetNumElements();
    desc.Buffer.Flags = msUAVFlag[sbuffer->GetCounterType()];

    HRESULT hr = device->CreateUnorderedAccessView(buffer, &desc, &mUAView);
    CHECK_HR_RETURN_NONE("Failed to create unordered access view");
}

void DX11StructuredBuffer::CreateCounterStaging(ID3D11Device* device)
{
    // This allows us to read the internal counter of the buffer (if it
    // has one).
    D3D11_BUFFER_DESC desc;
    desc.ByteWidth = 4;  // sizeof(unsigned int)
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;

    HRESULT hr = device->CreateBuffer(&desc, nullptr, &mCounterStaging);
    CHECK_HR_RETURN_NONE("Failed to create counter staging buffer");
}


unsigned int const DX11StructuredBuffer::msUAVFlag[] =
{
    D3D11_BUFFER_UAV_FLAG_BASIC,
    D3D11_BUFFER_UAV_FLAG_APPEND,
    D3D11_BUFFER_UAV_FLAG_COUNTER
};
