// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11IndirectArgumentsBuffer.h>
using namespace gte;

DX11IndirectArgumentsBuffer::DX11IndirectArgumentsBuffer(ID3D11Device* device, IndirectArgumentsBuffer const* iabuffer)
    :
    DX11Buffer(iabuffer)
{
    // Specify the counter buffer description.
    D3D11_BUFFER_DESC desc;
    desc.ByteWidth = iabuffer->GetNumBytes();
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
    desc.StructureByteStride = 0;

    // Create the counter buffer.
    ID3D11Buffer* buffer = nullptr;
    HRESULT hr;
    if (iabuffer->GetData())
    {
        D3D11_SUBRESOURCE_DATA data;
        data.pSysMem = iabuffer->GetData();
        data.SysMemPitch = 0;
        data.SysMemSlicePitch = 0;
        hr = device->CreateBuffer(&desc, &data, &buffer);
    }
    else
    {
        hr = device->CreateBuffer(&desc, nullptr, &buffer);
    }
    CHECK_HR_RETURN_NONE("Failed to create indirect-arguments buffer");
    mDXObject = buffer;
}

std::shared_ptr<GEObject> DX11IndirectArgumentsBuffer::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_INDIRECT_ARGUMENTS_BUFFER)
    {
        return std::make_shared<DX11IndirectArgumentsBuffer>(
            reinterpret_cast<ID3D11Device*>(device),
            static_cast<IndirectArgumentsBuffer const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}
